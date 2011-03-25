/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2009 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

#include <linux/version.h>
#include <linux/fb.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/amba/bus.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <asm/sizes.h>
#include <asm/uaccess.h>
#include <mach/motherboard.h>

//#define MALI_USE_UNIFIED_MEMORY_PROVIDER
#define MALI_HDLCD_RES 3

#ifdef MALI_USE_UNIFIED_MEMORY_PROVIDER
#include "ump_kernel_interface.h"
#include "ump_kernel_interface_ref_drv.h"
#endif

#define MALI_HDLCD_NAME "lt:hdlcd"
#define HDLCD_IOCTL_GET_FB_UMP_SECURE_ID  _IOWR('m',0xF8, __u32)

struct {
	struct fb_videomode	mode;
	signed short		width;	/* width in mm */
	signed short		height;	/* height in mm */
	unsigned int		bpp:8;
} default_settings =
#if (MALI_HDLCD_RES == 0)
{
	.mode = {
		.name		= "VGA",
		.refresh	= 60,
		.xres		= 640,
		.yres		= 480,
		.pixclock	= 39721,
		.left_margin	= 40,
		.right_margin	= 24,
		.upper_margin	= 32,
		.lower_margin	= 11,
		.hsync_len	= 96,
		.vsync_len	= 2,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.bpp		= 16,
	.width		= -1,
	.height		= -1,
};
#elif (MALI_HDLCD_RES == 1)
{
	.mode = {
		.name		= "DVI-SVGA",
		.refresh	= 60,
		.xres		= 800,
		.yres		= 600,
		.pixclock	= 26144,
		.hsync_len	= 80,
		.left_margin	= 112,
		.right_margin	= 32,
		.vsync_len	= 4,
		.upper_margin	= 18,
		.lower_margin	= 4,
		.vmode          = FB_VMODE_NONINTERLACED,
	},
	.bpp = 16,
	.width = -1,
	.height = 1,
};
#elif (MALI_HDLCD_RES == 2)
{
	.mode = {
		.name		= "SVGA",
		.refresh	= 60,
		.xres		= 1024,
		.yres		= 768,
		.pixclock	= 15748,
		.hsync_len	= 104,
		.left_margin	= 152,
		.right_margin	= 48,
		.vsync_len	= 4,
		.upper_margin	= 24,
		.lower_margin	= 4,
	},
	.bpp = 16,
	.width = -1,
	.height = 1,
};
#elif (MALI_HDLCD_RES == 3)
{
	.mode		= {
		.name           = "UXGA",
		.xres           = 1680,
		.yres           = 1050,
		.pixclock       = 8403,
		.left_margin    = 80,
		.right_margin   = 48,
		.hsync_len      = 32,
		.upper_margin   = 21,
		.lower_margin   = 3,
		.vsync_len      = 6,
		.vmode          = FB_VMODE_NONINTERLACED,
	},
	.bpp		= 16,
	.width		= -1,
	.height		= -1,
};
#else
{
	.mode		= {
		.name           = "HD",
		.xres           = 1920,
		.yres           = 1080,
		.pixclock       = 722,
		.hsync_len      = 80,
		.left_margin    = 48,
		.right_margin   = 160,
		.vsync_len      = 8,
		.upper_margin   = 31,
		.lower_margin   = 3,
		.vmode          = FB_VMODE_NONINTERLACED,
	},
	.bpp		= 16,
	.width		= -1,
	.height		= -1,
};
#endif

#ifdef MALI_USE_UNIFIED_MEMORY_PROVIDER
ump_dd_handle ump_wrapped_buffer;
#endif

enum
{
	HDLCD_REGISTER_VERSION          = 0x0000, /*ro*/
	HDLCD_REGISTER_INT_RAWSTAT      = 0x0010, /*rw*/
	HDLCD_REGISTER_INT_CLEAR        = 0x0014, /*wo*/
	HDLCD_REGISTER_INT_MASK         = 0x0018, /*rw*/
	HDLCD_REGISTER_INT_STATUS       = 0x001C, /*ro*/
	HDLCD_REGISTER_USER_OUT         = 0x0020, /*rw*/
	HDLCD_REGISTER_FB_BASE          = 0x0100, /*rw*/
	HDLCD_REGISTER_FB_LINE_LENGTH   = 0x0104, /*rw*/
	HDLCD_REGISTER_FB_LINE_COUNT    = 0x0108, /*rw*/
	HDLCD_REGISTER_FB_LINE_PITCH    = 0x010C, /*rw*/
	HDLCD_REGISTER_BUS_OPTIONS      = 0x0110, /*rw*/
	HDLCD_REGISTER_V_SYNC           = 0x0200, /*rw*/
	HDLCD_REGISTER_V_BACK_PORCH     = 0x0204, /*rw*/
	HDLCD_REGISTER_V_DATA           = 0x0208, /*rw*/
	HDLCD_REGISTER_V_FRONT_PORCH    = 0x020C, /*rw*/
	HDLCD_REGISTER_H_SYNC           = 0x0210, /*rw*/
	HDLCD_REGISTER_H_BACK_PORCH     = 0x0214, /*rw*/
	HDLCD_REGISTER_H_DATA           = 0x0218, /*rw*/
	HDLCD_REGISTER_H_FRONT_PORCH    = 0x021C, /*rw*/
	HDLCD_REGISTER_POLARITIES       = 0x0220, /*rw*/
	HDLCD_REGISTER_COMMAND          = 0x0230, /*rw*/
	HDLCD_REGISTER_PIXEL_FORMAT     = 0x0240, /*rw*/
	HDLCD_REGISTER_RED_SELECT       = 0x0244, /*rw*/
	HDLCD_REGISTER_GREEN_SELECT     = 0x0248, /*rw*/
	HDLCD_REGISTER_BLUE_SELECT      = 0x024C, /*rw*/
};

enum
{
	HDLCD_POLARITY_VSYNC      = 1 << 0,
	HDLCD_POLARITY_HSYNC      = 1 << 1,
	HDLCD_POLARITY_DATAEN     = 1 << 2,
	HDLCD_POLARITY_DATA       = 1 << 3,
	HDLCD_POLARITY_PXLCLK     = 1 << 4,
};

typedef struct hdlcd_device
{
	struct fb_info    fb;
	struct platform_device *dev;
	struct clk *clk;
	void __iomem *    base;
	u32               cmap[16];
} hdlcd_device;

typedef struct hdlcd_registers
{
	u32 user_out;

	u32 fb_line_length;
	u32 fb_line_count;
	u32 fb_line_pitch;

	u32 bus_options;

	u32 v_sync;
	u32 v_back_porch;
	u32 v_data;
	u32 v_front_porch;

	u32 h_sync;
	u32 h_back_porch;
	u32 h_data;
	u32 h_front_porch;

	u32 polarities;

	u32 pixel_format;
	u32 red_select;
	u32 green_select;
	u32 blue_select;

	u32 pixclock;
}hdlcd_registers;

static void hdlcd_disable(hdlcd_device *hdlcd)
{
	void __iomem *cmd_reg = hdlcd->base + HDLCD_REGISTER_COMMAND;
	u32 val;

	val = readl(cmd_reg);
	val &= ~1; /* turn off LSB */
	writel(val, cmd_reg);

	/* Disable clock source.  */
	clk_disable(hdlcd->clk);
}


#define VGA 0  /* 640x480 */
#define SVGA 1  /* 800x600 */
#define XGA 2 /* 1024x768 */
#define SXGA 3 /* 1280x1024 */
#define UXGA 4 /* 1600x1200 */
#define HD1080 5 /* 1920x1080 */

/*
 * Enable the relevant connector on the interface module.
 */
static void hdlcd_enable(hdlcd_device *hdlcd)
{
	void __iomem *cmd_reg = hdlcd->base + HDLCD_REGISTER_COMMAND;
	u32 val;

	/* Enable clock source.  */
	clk_enable(hdlcd->clk);

	val = 1; /* turn on LSB, let all others be zero */
	writel(val, cmd_reg);
}

static int hdlcd_setup(struct hdlcd_device *hdlcd)
{
	struct platform_device *pdev = hdlcd->dev;
	struct resource *res;
	dma_addr_t dma;
	/* Maximum resolution supported.  */
	unsigned long framesize = PAGE_ALIGN(1600 * 1200 * 2 * 2);

#ifdef MALI_USE_UNIFIED_MEMORY_PROVIDER
	ump_dd_physical_block ump_memory_description;
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_warning("Could not allocate resource.\n");
		return  -ENODEV;
	}
	printk("HDLCD MEM START @ [0x%x] SIZE[0x%x]\n", res->start, resource_size(res));

	if (NULL == request_mem_region(res->start, resource_size(res), "hdlcd"))
	{
		printk(KERN_ERR "Mali HDLCD: Failed to acquire the register space for the controller\n");
		return -EBUSY;
	}

	hdlcd->fb.screen_base = dma_alloc_writecombine(&hdlcd->dev->dev, framesize,
						      &dma, GFP_KERNEL);
	if (!hdlcd->fb.screen_base) {
		release_mem_region(res->start, resource_size(res));
		printk(KERN_ERR "HDLCD: unable to map frame buffer\n");
		return -ENOMEM;
	}
	hdlcd->fb.fix.smem_start = dma;
	hdlcd->fb.fix.smem_len = framesize;

	hdlcd->fb.fix.mmio_start  = res->start;
	hdlcd->fb.fix.mmio_len    = resource_size(res);

	/* Clear the fb.  */
	memset_io(hdlcd->fb.screen_base, 0, framesize);

#ifdef MALI_USE_UNIFIED_MEMORY_PROVIDER
	ump_memory_description.addr = dma;
	ump_memory_description.size = framesize;

	ump_wrapped_buffer = ump_dd_handle_create_from_phys_blocks(&ump_memory_description, 1);
	if (UMP_DD_HANDLE_INVALID == ump_wrapped_buffer)
	{
		dma_free_writecombine(&hdlcd->dev->dev, hdlcd->fb.fix.smem_len,
				      hdlcd->fb.screen_base, hdlcd->fb.fix.smem_start);
		release_mem_region(res->start, resource_size(res));
		printk(KERN_ERR "CLCD: Unable to wrap framebuffer as a ump handle\n");
		return -ENOMEM;
	}
#endif

	printk("Mali HDLCD: frame buffer virt base %p phys base 0x%lX\n", hdlcd->fb.screen_base, (unsigned long)dma);

	return 0;
}

static inline int hdlcd_check(hdlcd_device *dev, struct fb_var_screeninfo *var)
{
	var->xres_virtual = var->xres = (var->xres + 15) & ~15;
	var->yres = (var->yres + 1) & ~1;
	var->yres_virtual = 2 * var->yres;
	var->nonstd = 0;
	var->accel_flags = 0;

	return 0;

#define MALI_CHECK(e,l,h) (var->e < l || var->e > h)

	if (MALI_CHECK(right_margin, (5+1), 256) ||  /* back porch */
			MALI_CHECK(left_margin, (5+1), 256) ||   /* front porch */
			MALI_CHECK(hsync_len, (5+1), 256) ||
			var->xres > 4096 ||
			var->lower_margin > 255 ||          /* back porch */
			var->upper_margin > 255 ||          /* front porch */
			var->vsync_len > 32 ||
			var->yres > 1024)
		return -EINVAL;
#undef MALI_CHECK

	var->nonstd = 0;
	var->accel_flags = 0;

	return 0;
}

static inline void hdlcd_decode(hdlcd_device *dev, hdlcd_registers *regs)
{
	regs->fb_line_length = dev->fb.var.xres * dev->fb.var.bits_per_pixel/8;
	regs->fb_line_count = dev->fb.var.yres - 1;
	regs->fb_line_pitch = regs->fb_line_length;

	regs->v_sync = dev->fb.var.vsync_len - 1;
	regs->v_back_porch = dev->fb.var.upper_margin - 1;
	regs->v_data = dev->fb.var.yres - 1;
	regs->v_front_porch = dev->fb.var.lower_margin - 1;

	regs->h_sync = dev->fb.var.hsync_len - 1;
	regs->h_back_porch = dev->fb.var.left_margin - 1;
	regs->h_data = dev->fb.var.xres - 1;
	regs->h_front_porch = dev->fb.var.right_margin - 1;

	regs->polarities = HDLCD_POLARITY_PXLCLK | HDLCD_POLARITY_DATAEN | HDLCD_POLARITY_DATA;
	regs->polarities |= (dev->fb.var.sync & FB_SYNC_HOR_HIGH_ACT) ? HDLCD_POLARITY_HSYNC : 0;
	regs->polarities |= (dev->fb.var.sync & FB_SYNC_VERT_HIGH_ACT) ? HDLCD_POLARITY_VSYNC : 0;

	regs->pixel_format = ((dev->fb.var.bits_per_pixel / 8) - 1) << 3;

	/* Original colors.  */
#if 0
	regs->red_select    = ((dev->fb.var.red.length & 0xF) << 8  ) | dev->fb.var.red.offset;
	regs->green_select  = ((dev->fb.var.green.length & 0xF) << 8  ) | dev->fb.var.green.offset;
	regs->blue_select   = ((dev->fb.var.blue.length & 0xF) << 8  ) | dev->fb.var.blue.offset;
#else
	regs->red_select    = ((dev->fb.var.blue.length & 0xF) << 8  ) | dev->fb.var.blue.offset;
	regs->green_select  = ((dev->fb.var.green.length & 0xF) << 8  ) | dev->fb.var.green.offset;
	regs->blue_select   = ((dev->fb.var.red.length & 0xF) << 8  ) | dev->fb.var.red.offset;
#endif

	regs->pixclock = dev->fb.var.pixclock;
}


#define to_device(info)	container_of(info, hdlcd_device, fb)


static inline void hdlcd_set_start(hdlcd_device *hdlcd, u32 offset)
{
	unsigned long start = hdlcd->fb.fix.smem_start;

	start += hdlcd->fb.var.yoffset * hdlcd->fb.fix.line_length;
	writel(start, hdlcd->base + HDLCD_REGISTER_FB_BASE);
}

static int hdlcd_set_bitfields(hdlcd_device *dev, struct fb_var_screeninfo *var)
{
	int ret = 0;

	memset(&var->transp, 0, sizeof(var->transp));

	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;

	switch (var->bits_per_pixel)
	{
		case 1:
		case 2:
		case 4:
			ret = -EINVAL;
			break;
		case 8:
			var->red.length = 3;
			var->green.length = 3;
			var->blue.length = 2;
			break;
		case 16:
			var->red.length      = 5;
			var->blue.length     = 5;
			if (var->green.length != 5 && var->green.length != 6) var->green.length = 6;
			break;
		case 24:
		case 32:
			var->red.length      = 8;
			var->green.length    = 8;
			var->blue.length     = 8;
			break;
		default:
			ret = -EINVAL;
			break;
	}

	if (ret == 0)
	{
		var->blue.offset = 0;
		var->green.offset = var->blue.offset + var->blue.length;
		var->red.offset = var->green.offset + var->green.length;
	}

	return ret;
}

static int hdlcd_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	hdlcd_device *dev = to_device(info);
	int ret = 0;

	var->yres_virtual = 2 * var->yres;

	if ((var->xres_virtual * var->bits_per_pixel / 8 * var->yres_virtual) > dev->fb.fix.smem_len) ret = -EINVAL;

	if (ret == 0) ret = hdlcd_set_bitfields(dev, var);

	return ret;
}

static int hdlcd_set_par(struct fb_info *info)
{
	hdlcd_device *dev = to_device(info);
	struct hdlcd_registers regs;

	dev->fb.fix.line_length = dev->fb.var.xres_virtual * dev->fb.var.bits_per_pixel / 8;

	dev->fb.fix.visual = FB_VISUAL_TRUECOLOR;

	hdlcd_decode(dev, &regs);

	hdlcd_disable(dev);

	writel(regs.fb_line_length,      dev->base + HDLCD_REGISTER_FB_LINE_LENGTH);
	writel(regs.fb_line_count,       dev->base + HDLCD_REGISTER_FB_LINE_COUNT);
	writel(regs.fb_line_pitch,       dev->base + HDLCD_REGISTER_FB_LINE_PITCH);
	writel(0xF10,                    dev->base + HDLCD_REGISTER_BUS_OPTIONS);
	writel(regs.v_sync,              dev->base + HDLCD_REGISTER_V_SYNC);
	writel(regs.v_back_porch,        dev->base + HDLCD_REGISTER_V_BACK_PORCH);
	writel(regs.v_data,              dev->base + HDLCD_REGISTER_V_DATA);
	writel(regs.v_front_porch,       dev->base + HDLCD_REGISTER_V_FRONT_PORCH);
	writel(regs.h_sync,              dev->base + HDLCD_REGISTER_H_SYNC);
	writel(regs.h_back_porch,        dev->base + HDLCD_REGISTER_H_BACK_PORCH);
	writel(regs.h_data,              dev->base + HDLCD_REGISTER_H_DATA);
	writel(regs.h_front_porch,       dev->base + HDLCD_REGISTER_H_FRONT_PORCH);
	writel(regs.polarities,          dev->base + HDLCD_REGISTER_POLARITIES);
	writel(regs.pixel_format,        dev->base + HDLCD_REGISTER_PIXEL_FORMAT);
	writel(regs.red_select,          dev->base + HDLCD_REGISTER_RED_SELECT);
	writel(regs.green_select,        dev->base + HDLCD_REGISTER_GREEN_SELECT);
	writel(regs.blue_select,         dev->base + HDLCD_REGISTER_BLUE_SELECT);

	hdlcd_set_start(dev, 0);

	clk_set_rate(dev->clk, (1000000000 / regs.pixclock) * 1000);

	hdlcd_enable(dev);

	return 0;
}

/* */
static inline u32 convert_bitfield(int val, struct fb_bitfield *bf)
{
	unsigned int mask = (1 << bf->length) - 1;

	return (val >> (16 - bf->length) & mask) << bf->offset;
}

/*
 *  Set a single color register. The values supplied have a 16 bit
 *  magnitude.  Return != 0 for invalid regno.
 */
static int hdlcd_setcolreg(unsigned int regno, unsigned int red, unsigned int green,
		unsigned int blue, unsigned int transp, struct fb_info *info)
{
	hdlcd_device *dev = to_device(info);

	if (regno < 16)
	{
		dev->cmap[regno] = convert_bitfield(transp, &dev->fb.var.transp) |
			convert_bitfield(blue, &dev->fb.var.blue) |
			convert_bitfield(green, &dev->fb.var.green) |
			convert_bitfield(red, &dev->fb.var.red);

		return 0;
	}

	return -1;
}

/*
 *  Blank the screen if blank_mode != 0, else unblank. If blank == NULL
 *  then the caller blanks by setting the CLUT (Color Look Up Table) to all
 *  black. Return 0 if blanking succeeded, != 0 if un-/blanking failed due
 *  to e.g. a video mode which doesn't support it. Implements VESA suspend
 *  and powerdown modes on hardware that supports disabling hsync/vsync:
 *    blank_mode == 2: suspend vsync
 *    blank_mode == 3: suspend hsync
 *    blank_mode == 4: powerdown
 */
static int hdlcd_blank(int blank_mode, struct fb_info *info)
{
	hdlcd_device *dev = to_device(info);

	if (blank_mode != 0) {
		hdlcd_disable(dev);
	} else {
		hdlcd_enable(dev);
	}
	return 0;
}

static int hdlcd_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
#if 0
	pgprot_t prot;
	hdlcd_device *dev = to_device(info);
	unsigned long len, off = vma->vm_pgoff << PAGE_SHIFT;

	len = info->fix.smem_len;

	if (off <= len && vma->vm_end - vma->vm_start <= len - off)
	{
		vma->vm_flags |= VM_IO;
		vma->vm_flags |= VM_RESERVED;

		prot = pgprot_writecombine(vma->vm_page_prot);

		return io_remap_pfn_range(vma, vma->vm_start, ((unsigned long)dev->fb.fix.smem_start) >> PAGE_SHIFT, vma->vm_end - vma->vm_start, prot);
	}

	return -EINVAL;
#endif

	struct hdlcd_device *hdlcd = container_of(info, struct hdlcd_device, fb);

	return dma_mmap_writecombine(&hdlcd->dev->dev, vma, hdlcd->fb.screen_base,
				     hdlcd->fb.fix.smem_start, hdlcd->fb.fix.smem_len);
}

static int hdlcd_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	hdlcd_device *dev = to_device(info);
	dev->fb.var.yoffset = var->yoffset;
	hdlcd_set_start(dev, dev->fb.var.yoffset * dev->fb.fix.line_length);
	return 0;
}

static int hdlcd_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
#ifdef MALI_USE_UNIFIED_MEMORY_PROVIDER
	if ( HDLCD_IOCTL_GET_FB_UMP_SECURE_ID == cmd )
	{
		u32 __user * psecureid = (u32 __user *)arg;
		ump_secure_id secure_id = ump_dd_secure_id_get(ump_wrapped_buffer);
		return put_user( (u32)secure_id, psecureid);
	}
#endif

	return -ENOIOCTLCMD;
}


static struct fb_ops hdlcd_ops =
{
	.owner            = THIS_MODULE,
	.fb_check_var     = hdlcd_check_var,
	.fb_set_par       = hdlcd_set_par,
	.fb_setcolreg     = hdlcd_setcolreg,
	.fb_blank         = hdlcd_blank,
	.fb_fillrect      = cfb_fillrect,
	.fb_copyarea      = cfb_copyarea,
	.fb_imageblit     = cfb_imageblit,
	.fb_mmap          = hdlcd_mmap,
	.fb_pan_display   = hdlcd_pan_display,
	.fb_ioctl         = hdlcd_ioctl,
	.fb_compat_ioctl  = hdlcd_ioctl
};

static int hdlcd_register(struct hdlcd_device *hdlcd)
{
	int ret;

	hdlcd->clk = clk_get(&hdlcd->dev->dev, NULL);
	if (IS_ERR(hdlcd->clk)) {
		ret = PTR_ERR(hdlcd->clk);
		goto out;
	}

	hdlcd->base = ioremap_nocache(hdlcd->fb.fix.mmio_start, hdlcd->fb.fix.mmio_len);
	if (!hdlcd->base)
	{
		printk(KERN_ERR "Mali HDLCD: unable to remap registers\n");
		clk_put(hdlcd->clk);
		return -ENOMEM;
	}

	hdlcd->fb.fbops              = &hdlcd_ops;
	hdlcd->fb.flags              = FBINFO_FLAG_DEFAULT;
	hdlcd->fb.pseudo_palette     = hdlcd->cmap;

	strncpy(hdlcd->fb.fix.id, MALI_HDLCD_NAME, sizeof(hdlcd->fb.fix.id));
	hdlcd->fb.fix.type           = FB_TYPE_PACKED_PIXELS;
	hdlcd->fb.fix.type_aux       = 0;
	hdlcd->fb.fix.xpanstep       = 0;
	hdlcd->fb.fix.ypanstep       = 1;
	hdlcd->fb.fix.ywrapstep      = 0;
	hdlcd->fb.fix.accel          = FB_ACCEL_NONE;

	hdlcd->fb.var.xres           = default_settings.mode.xres;
	hdlcd->fb.var.yres           = default_settings.mode.yres;
	hdlcd->fb.var.xres_virtual   = default_settings.mode.xres;
	hdlcd->fb.var.yres_virtual   = default_settings.mode.yres * 2;
	hdlcd->fb.var.bits_per_pixel = default_settings.bpp;
	hdlcd->fb.var.grayscale      = 0;
	hdlcd->fb.var.pixclock       = default_settings.mode.pixclock;
	hdlcd->fb.var.left_margin    = default_settings.mode.left_margin;
	hdlcd->fb.var.right_margin   = default_settings.mode.right_margin;
	hdlcd->fb.var.upper_margin   = default_settings.mode.upper_margin;
	hdlcd->fb.var.lower_margin   = default_settings.mode.lower_margin;
	hdlcd->fb.var.hsync_len      = default_settings.mode.hsync_len;
	hdlcd->fb.var.vsync_len      = default_settings.mode.vsync_len;
	hdlcd->fb.var.sync           = default_settings.mode.sync;
	hdlcd->fb.var.vmode          = default_settings.mode.vmode;
	hdlcd->fb.var.activate       = FB_ACTIVATE_NOW;
	hdlcd->fb.var.nonstd         = 0;
	hdlcd->fb.var.height         = default_settings.height;
	hdlcd->fb.var.width          = default_settings.width;
	hdlcd->fb.var.accel_flags    = 0;

	hdlcd->fb.monspecs.hfmin	= 0;
	hdlcd->fb.monspecs.hfmax   = 100000;
	hdlcd->fb.monspecs.vfmin	= 0;
	hdlcd->fb.monspecs.vfmax	= 400;
	hdlcd->fb.monspecs.dclkmin = 1000000;
	hdlcd->fb.monspecs.dclkmax	= 100000000;

	/*
	 * Make sure that the bitfields are set appropriately.
	 */
	hdlcd_set_bitfields(hdlcd, &hdlcd->fb.var);

	/*
	 * Ensure interrupts are disabled.
	 */
	/* writel(0, hdlcd->base + 0x18);  */

	fb_set_var(&hdlcd->fb, &hdlcd->fb.var);
	if (!register_framebuffer(&hdlcd->fb))
		return 0;

	iounmap(hdlcd->base);
out:
	printk(KERN_ERR "HDLCD: cannot register framebuffer\n");
	return -EFAULT;
}

static int hdlcd_drv_remove(struct platform_device *pdev)
{
	hdlcd_device *hdlcd = platform_get_drvdata(pdev);

	hdlcd_disable(hdlcd);
	unregister_framebuffer(&hdlcd->fb);

#ifdef MALI_USE_UNIFIED_MEMORY_PROVIDER
	ump_dd_reference_release(ump_wrapped_buffer);
	ump_wrapped_buffer = UMP_DD_HANDLE_INVALID;
#endif

	/* Unmap and release registers.  */
	iounmap(hdlcd->base);
	release_mem_region(hdlcd->fb.fix.mmio_start, hdlcd->fb.fix.mmio_len);

	/* Deallocate fb memory.  */
	dma_free_writecombine(&hdlcd->dev->dev, hdlcd->fb.fix.smem_len,
			      hdlcd->fb.screen_base, hdlcd->fb.fix.smem_start);

	clk_put(hdlcd->clk);

	kfree(hdlcd);

	return 0;
}

static int __devinit hdlcd_drv_probe(struct platform_device *pdev)
{
	hdlcd_device *hdlcd;

	hdlcd = (hdlcd_device *)kzalloc(sizeof(*hdlcd), GFP_KERNEL);
	if (!hdlcd)
		return -ENOMEM;

	hdlcd->dev = pdev;

	if (hdlcd_setup(hdlcd)) {
		kfree(hdlcd);
		return -EFAULT;
	}

	if (hdlcd_register(hdlcd)) {
		kfree(hdlcd);
		return -EFAULT;
	}
	platform_set_drvdata(pdev, hdlcd);

	return 0;
}
#ifdef CONFIG_OF
static struct of_device_id hdlcd_matches[] = {
	{ .compatible = "arm,hdlcd"},
	{},
};
#endif
static struct platform_driver hdcld_driver = {
	.probe = hdlcd_drv_probe,
	.remove = __devexit_p(hdlcd_drv_remove),
	.driver = {
		.name 	= MALI_HDLCD_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = hdlcd_matches,
#endif
	},
};

static int __init hdlcd_init(void)
{
	return platform_driver_register(&hdcld_driver);
}

static void __exit hdlcd_exit(void)
{
	platform_driver_unregister(&hdcld_driver);
}

module_init(hdlcd_init);
module_exit(hdlcd_exit);

MODULE_DESCRIPTION("ARM Mali HDLCD core driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("John Einar Reitan");
