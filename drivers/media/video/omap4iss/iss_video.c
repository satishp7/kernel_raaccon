/*
 * TI OMAP4 ISS V4L2 Driver - Generic video node
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 *
 * Author: Sergio Aguirre <saaguirre@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/cacheflush.h>
#include <linux/clk.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <plat/omap-pm.h>

#include "iss_video.h"
#include "iss.h"


/* -----------------------------------------------------------------------------
 * Helper functions
 */

static struct iss_format_info formats[] = {
	{ V4L2_MBUS_FMT_Y8_1X8, V4L2_MBUS_FMT_Y8_1X8,
	  V4L2_MBUS_FMT_Y8_1X8, V4L2_MBUS_FMT_Y8_1X8,
	  V4L2_PIX_FMT_GREY, 8, },
	{ V4L2_MBUS_FMT_Y10_1X10, V4L2_MBUS_FMT_Y10_1X10,
	  V4L2_MBUS_FMT_Y10_1X10, V4L2_MBUS_FMT_Y8_1X8,
	  V4L2_PIX_FMT_Y10, 10, },
	{ V4L2_MBUS_FMT_Y12_1X12, V4L2_MBUS_FMT_Y10_1X10,
	  V4L2_MBUS_FMT_Y12_1X12, V4L2_MBUS_FMT_Y8_1X8,
	  V4L2_PIX_FMT_Y12, 12, },
	{ V4L2_MBUS_FMT_SBGGR8_1X8, V4L2_MBUS_FMT_SBGGR8_1X8,
	  V4L2_MBUS_FMT_SBGGR8_1X8, V4L2_MBUS_FMT_SBGGR8_1X8,
	  V4L2_PIX_FMT_SBGGR8, 8, },
	{ V4L2_MBUS_FMT_SGBRG8_1X8, V4L2_MBUS_FMT_SGBRG8_1X8,
	  V4L2_MBUS_FMT_SGBRG8_1X8, V4L2_MBUS_FMT_SGBRG8_1X8,
	  V4L2_PIX_FMT_SGBRG8, 8, },
	{ V4L2_MBUS_FMT_SGRBG8_1X8, V4L2_MBUS_FMT_SGRBG8_1X8,
	  V4L2_MBUS_FMT_SGRBG8_1X8, V4L2_MBUS_FMT_SGRBG8_1X8,
	  V4L2_PIX_FMT_SGRBG8, 8, },
	{ V4L2_MBUS_FMT_SRGGB8_1X8, V4L2_MBUS_FMT_SRGGB8_1X8,
	  V4L2_MBUS_FMT_SRGGB8_1X8, V4L2_MBUS_FMT_SRGGB8_1X8,
	  V4L2_PIX_FMT_SRGGB8, 8, },
	{ V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8, V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8,
	  V4L2_MBUS_FMT_SGRBG10_1X10, 0,
	  V4L2_PIX_FMT_SGRBG10DPCM8, 8, },
	{ V4L2_MBUS_FMT_SBGGR10_1X10, V4L2_MBUS_FMT_SBGGR10_1X10,
	  V4L2_MBUS_FMT_SBGGR10_1X10, V4L2_MBUS_FMT_SBGGR8_1X8,
	  V4L2_PIX_FMT_SBGGR10, 10, },
	{ V4L2_MBUS_FMT_SGBRG10_1X10, V4L2_MBUS_FMT_SGBRG10_1X10,
	  V4L2_MBUS_FMT_SGBRG10_1X10, V4L2_MBUS_FMT_SGBRG8_1X8,
	  V4L2_PIX_FMT_SGBRG10, 10, },
	{ V4L2_MBUS_FMT_SGRBG10_1X10, V4L2_MBUS_FMT_SGRBG10_1X10,
	  V4L2_MBUS_FMT_SGRBG10_1X10, V4L2_MBUS_FMT_SGRBG8_1X8,
	  V4L2_PIX_FMT_SGRBG10, 10, },
	{ V4L2_MBUS_FMT_SRGGB10_1X10, V4L2_MBUS_FMT_SRGGB10_1X10,
	  V4L2_MBUS_FMT_SRGGB10_1X10, V4L2_MBUS_FMT_SRGGB8_1X8,
	  V4L2_PIX_FMT_SRGGB10, 10, },
	{ V4L2_MBUS_FMT_SBGGR12_1X12, V4L2_MBUS_FMT_SBGGR10_1X10,
	  V4L2_MBUS_FMT_SBGGR12_1X12, V4L2_MBUS_FMT_SBGGR8_1X8,
	  V4L2_PIX_FMT_SBGGR12, 12, },
	{ V4L2_MBUS_FMT_SGBRG12_1X12, V4L2_MBUS_FMT_SGBRG10_1X10,
	  V4L2_MBUS_FMT_SGBRG12_1X12, V4L2_MBUS_FMT_SGBRG8_1X8,
	  V4L2_PIX_FMT_SGBRG12, 12, },
	{ V4L2_MBUS_FMT_SGRBG12_1X12, V4L2_MBUS_FMT_SGRBG10_1X10,
	  V4L2_MBUS_FMT_SGRBG12_1X12, V4L2_MBUS_FMT_SGRBG8_1X8,
	  V4L2_PIX_FMT_SGRBG12, 12, },
	{ V4L2_MBUS_FMT_SRGGB12_1X12, V4L2_MBUS_FMT_SRGGB10_1X10,
	  V4L2_MBUS_FMT_SRGGB12_1X12, V4L2_MBUS_FMT_SRGGB8_1X8,
	  V4L2_PIX_FMT_SRGGB12, 12, },
	{ V4L2_MBUS_FMT_UYVY8_1X16, V4L2_MBUS_FMT_UYVY8_1X16,
	  V4L2_MBUS_FMT_UYVY8_1X16, 0,
	  V4L2_PIX_FMT_UYVY, 16, },
	{ V4L2_MBUS_FMT_YUYV8_1X16, V4L2_MBUS_FMT_YUYV8_1X16,
	  V4L2_MBUS_FMT_YUYV8_1X16, 0,
	  V4L2_PIX_FMT_YUYV, 16, },
};

const struct iss_format_info *
omap4iss_video_format_info(enum v4l2_mbus_pixelcode code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (formats[i].code == code)
			return &formats[i];
	}

	return NULL;
}

/*
 * iss_video_mbus_to_pix - Convert v4l2_mbus_framefmt to v4l2_pix_format
 * @video: ISS video instance
 * @mbus: v4l2_mbus_framefmt format (input)
 * @pix: v4l2_pix_format format (output)
 *
 * Fill the output pix structure with information from the input mbus format.
 * The bytesperline and sizeimage fields are computed from the requested bytes
 * per line value in the pix format and information from the video instance.
 *
 * Return the number of padding bytes at end of line.
 */
static unsigned int iss_video_mbus_to_pix(const struct iss_video *video,
					  const struct v4l2_mbus_framefmt *mbus,
					  struct v4l2_pix_format *pix)
{
	unsigned int bpl = pix->bytesperline;
	unsigned int min_bpl;
	unsigned int i;

	memset(pix, 0, sizeof(*pix));
	pix->width = mbus->width;
	pix->height = mbus->height;

	/* Skip the last format in the loop so that it will be selected if no
	 * match is found.
	 */
	for (i = 0; i < ARRAY_SIZE(formats) - 1; ++i) {
		if (formats[i].code == mbus->code)
			break;
	}

	min_bpl = pix->width * ALIGN(formats[i].bpp, 8) / 8;

	/* Clamp the requested bytes per line value. If the maximum bytes per
	 * line value is zero, the module doesn't support user configurable line
	 * sizes. Override the requested value with the minimum in that case.
	 */
	if (video->bpl_max)
		bpl = clamp(bpl, min_bpl, video->bpl_max);
	else
		bpl = min_bpl;

	if (!video->bpl_zero_padding || bpl != min_bpl)
		bpl = ALIGN(bpl, video->bpl_alignment);

	pix->pixelformat = formats[i].pixelformat;
	pix->bytesperline = bpl;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = mbus->colorspace;
	pix->field = mbus->field;

	return bpl - min_bpl;
}

static void iss_video_pix_to_mbus(const struct v4l2_pix_format *pix,
				  struct v4l2_mbus_framefmt *mbus)
{
	unsigned int i;

	memset(mbus, 0, sizeof(*mbus));
	mbus->width = pix->width;
	mbus->height = pix->height;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (formats[i].pixelformat == pix->pixelformat)
			break;
	}

	if (WARN_ON(i == ARRAY_SIZE(formats)))
		return;

	mbus->code = formats[i].code;
	mbus->colorspace = pix->colorspace;
	mbus->field = pix->field;
}

static struct v4l2_subdev *
iss_video_remote_subdev(struct iss_video *video, u32 *pad)
{
	struct media_pad *remote;

	remote = media_entity_remote_source(&video->pad);

	if (remote == NULL )
		return NULL;

	   if( media_entity_type(remote->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
		return NULL;

	if (pad)
		*pad = remote->index;

    return media_entity_to_v4l2_subdev(remote->entity);
}

/* Return a pointer to the ISS video instance at the far end of the pipeline. */
static struct iss_video *
iss_video_far_end(struct iss_video *video)
{
	struct media_entity_graph graph;
	struct media_entity *entity = &video->video.entity;
	struct media_device *mdev = entity->parent;
	struct iss_video *far_end = NULL;

	mutex_lock(&mdev->graph_mutex);
	media_entity_graph_walk_start(&graph, entity);

	while ((entity = media_entity_graph_walk_next(&graph))) {
		if (entity == &video->video.entity)
			continue;

		if (media_entity_type(entity) != MEDIA_ENT_T_DEVNODE)
			continue;

		far_end = to_iss_video(media_entity_to_video_device(entity));
		if (far_end->type != video->type)
			break;

		far_end = NULL;
	}

	mutex_unlock(&mdev->graph_mutex);
	return far_end;
}

static int
__iss_video_get_format(struct iss_video *video, struct v4l2_format *format)
{
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = iss_video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	mutex_lock(&video->mutex);

	fmt.pad = pad;
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);
	if (ret == -ENOIOCTLCMD)
		ret = -EINVAL;

	mutex_unlock(&video->mutex);

	if (ret)
		return ret;

	format->type = video->type;
	return iss_video_mbus_to_pix(video, &fmt.format, &format->fmt.pix);
}

static int
iss_video_check_format(struct iss_video *video, struct iss_video_fh *vfh)
{
	struct v4l2_format format;
	int ret;

	memcpy(&format, &vfh->format, sizeof(format));
	ret = __iss_video_get_format(video, &format);
	if (ret < 0)
		return ret;

  printk("%s:%d vfh->format.fmt.pix.pixelformat - %x, format.fmt.pix.pixelformat - %x, vfh->format.fmt.pix.height - %x, format.fmt.pix.height - %x, vfh->format.fmt.pix.width - %x, format.fmt.pix.width - %x, vfh->format.fmt.pix.bytesperline - %x, format.fmt.pix.bytesperline - %x, vfh->format.fmt.pix.sizeimage - %x, format.fmt.pix.sizeimage - %x\n",
                       __func__, __LINE__,
                       vfh->format.fmt.pix.pixelformat, format.fmt.pix.pixelformat,
                       vfh->format.fmt.pix.height, format.fmt.pix.height,
                       vfh->format.fmt.pix.width, format.fmt.pix.width,
                       vfh->format.fmt.pix.bytesperline, format.fmt.pix.bytesperline,
                       vfh->format.fmt.pix.sizeimage, format.fmt.pix.sizeimage);

	if (vfh->format.fmt.pix.pixelformat != format.fmt.pix.pixelformat ||
	    vfh->format.fmt.pix.height != format.fmt.pix.height ||
	    vfh->format.fmt.pix.width != format.fmt.pix.width ||
	    vfh->format.fmt.pix.bytesperline != format.fmt.pix.bytesperline ||
	    vfh->format.fmt.pix.sizeimage != format.fmt.pix.sizeimage)
		return -EINVAL;

	return ret;
}

/* -----------------------------------------------------------------------------
 * Video queue operations
 */
//static int iss_video_queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
//static int iss_video_queue_setup(struct vb2_queue *vq, unsigned int *count, unsigned int *num_planes,
//				 unsigned int sizes[], void *alloc_ctxs[])
static int iss_video_queue_setup(struct vb2_queue *vq, unsigned int *count,
		unsigned int *num_planes, unsigned long sizes[],
		void *alloc_ctxs[])
{
	struct iss_video_fh *vfh = vb2_get_drv_priv(vq);
	struct iss_video *video = vfh->video;

	/* Revisit multi-planar support for NV12 */
	*num_planes = 1;

	sizes[0] = vfh->format.fmt.pix.sizeimage;
	if (sizes[0] == 0)
		return -EINVAL;

	alloc_ctxs[0] = video->alloc_ctx;

	*count = min(*count, (unsigned int)(video->capture_mem / PAGE_ALIGN(sizes[0])));

	return 0;
}

static void iss_video_buf_cleanup(struct vb2_buffer *vb)
{
	struct iss_buffer *buffer = container_of(vb, struct iss_buffer, vb);

	if (buffer->iss_addr)
		buffer->iss_addr = 0;
}

static int iss_video_buf_prepare(struct vb2_buffer *vb)
{
	struct iss_video_fh *vfh = vb2_get_drv_priv(vb->vb2_queue);
	struct iss_buffer *buffer = container_of(vb, struct iss_buffer, vb);
	struct iss_video *video = vfh->video;
	unsigned long size = vfh->format.fmt.pix.sizeimage;
	dma_addr_t addr;

	if (vb2_plane_size(vb, 0) < size)
		return -ENOBUFS;

	addr = vb2_dma_contig_plane_paddr(vb, 0);
	if (!IS_ALIGNED(addr, 32)) {
		dev_dbg(video->iss->dev, "Buffer address must be "
			"aligned to 32 bytes boundary.\n");
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);
	buffer->iss_addr = addr;
	return 0;
}

static void iss_video_buf_queue(struct vb2_buffer *vb)
{
	struct iss_video_fh *vfh = vb2_get_drv_priv(vb->vb2_queue);
	struct iss_video *video = vfh->video;
	struct iss_buffer *buffer = container_of(vb, struct iss_buffer, vb);
	struct iss_pipeline *pipe = to_iss_pipeline(&video->video.entity);
	unsigned int empty;
	unsigned long flags;

	spin_lock_irqsave(&video->qlock, flags);
	empty = list_empty(&video->dmaqueue);
	list_add_tail(&buffer->list, &video->dmaqueue);
	spin_unlock_irqrestore(&video->qlock, flags);

	if (empty) {
		enum iss_pipeline_state state;
		unsigned int start;

		if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			state = ISS_PIPELINE_QUEUE_OUTPUT;
		else
			state = ISS_PIPELINE_QUEUE_INPUT;

		spin_lock_irqsave(&pipe->lock, flags);
		pipe->state |= state;
		video->ops->queue(video, buffer);
		video->dmaqueue_flags |= ISS_VIDEO_DMAQUEUE_QUEUED;

		start = iss_pipeline_ready(pipe);
		if (start)
			pipe->state |= ISS_PIPELINE_STREAM;
		spin_unlock_irqrestore(&pipe->lock, flags);

		if (start)
			omap4iss_pipeline_set_stream(pipe,
						ISS_PIPELINE_STREAM_SINGLESHOT);
	}
}

static struct vb2_ops iss_video_vb2ops = {
	.queue_setup	= iss_video_queue_setup,
	.buf_prepare	= iss_video_buf_prepare,
	.buf_queue	= iss_video_buf_queue,
	.buf_cleanup	= iss_video_buf_cleanup,
};

/*
 * omap4iss_video_buffer_next - Complete the current buffer and return the next
 * @video: ISS video object
 *
 * Remove the current video buffer from the DMA queue and fill its timestamp,
 * field count and state fields before waking up its completion handler.
 *
 * For capture video nodes, the buffer state is set to VB2_BUF_STATE_DONE if no
 * error has been flagged in the pipeline, or to VB2_BUF_STATE_ERROR otherwise.
 *
 * The DMA queue is expected to contain at least one buffer.
 *
 * Return a pointer to the next buffer in the DMA queue, or NULL if the queue is
 * empty.
 */
struct iss_buffer *omap4iss_video_buffer_next(struct iss_video *video)
{
	struct iss_pipeline *pipe = to_iss_pipeline(&video->video.entity);
	enum iss_pipeline_state state;
	struct iss_buffer *buf;
	unsigned long flags;
	struct timespec ts;

	spin_lock_irqsave(&video->qlock, flags);
	if (WARN_ON(list_empty(&video->dmaqueue))) {
		spin_unlock_irqrestore(&video->qlock, flags);
		return NULL;
	}

	buf = list_first_entry(&video->dmaqueue, struct iss_buffer,
			       list);
	list_del(&buf->list);
	spin_unlock_irqrestore(&video->qlock, flags);

	ktime_get_ts(&ts);
	buf->vb.v4l2_buf.timestamp.tv_sec = ts.tv_sec;
	buf->vb.v4l2_buf.timestamp.tv_usec = ts.tv_nsec / NSEC_PER_USEC;

	/* Do frame number propagation only if this is the output video node.
	 * Frame number either comes from the CSI receivers or it gets
	 * incremented here if H3A is not active.
	 * Note: There is no guarantee that the output buffer will finish
	 * first, so the input number might lag behind by 1 in some cases.
	 */
	if (video == pipe->output && !pipe->do_propagation)
		buf->vb.v4l2_buf.sequence = atomic_inc_return(&pipe->frame_number);
	else
		buf->vb.v4l2_buf.sequence = atomic_read(&pipe->frame_number);

	vb2_buffer_done(&buf->vb, pipe->error ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);

	spin_lock_irqsave(&video->qlock, flags);
	if (list_empty(&video->dmaqueue)) {
		spin_unlock_irqrestore(&video->qlock, flags);
		if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			state = ISS_PIPELINE_QUEUE_OUTPUT
			      | ISS_PIPELINE_STREAM;
		else
			state = ISS_PIPELINE_QUEUE_INPUT
			      | ISS_PIPELINE_STREAM;

		spin_lock_irqsave(&pipe->lock, flags);
		pipe->state &= ~state;
		if (video->pipe.stream_state == ISS_PIPELINE_STREAM_CONTINUOUS)
			video->dmaqueue_flags |= ISS_VIDEO_DMAQUEUE_UNDERRUN;
		spin_unlock_irqrestore(&pipe->lock, flags);
		return NULL;
	}

	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE && pipe->input != NULL) {
		spin_lock_irqsave(&pipe->lock, flags);
		pipe->state &= ~ISS_PIPELINE_STREAM;
		spin_unlock_irqrestore(&pipe->lock, flags);
	}

	buf = list_first_entry(&video->dmaqueue, struct iss_buffer,
			       list);
	spin_unlock_irqrestore(&video->qlock, flags);
	buf->vb.state = VB2_BUF_STATE_ACTIVE;
	return buf;
}

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int
iss_video_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct iss_video *video = video_drvdata(file);

	strlcpy(cap->driver, ISS_VIDEO_DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, video->video.name, sizeof(cap->card));
	strlcpy(cap->bus_info, "media", sizeof(cap->bus_info));

	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	else
		cap->capabilities = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING;

	return 0;
}

static int
iss_video_get_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct iss_video_fh *vfh = to_iss_video_fh(fh);
	struct iss_video *video = video_drvdata(file);

	if (format->type != video->type)
		return -EINVAL;

	mutex_lock(&video->mutex);
	*format = vfh->format;
	mutex_unlock(&video->mutex);

	return 0;
}

static int
iss_video_set_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct iss_video_fh *vfh = to_iss_video_fh(fh);
	struct iss_video *video = video_drvdata(file);
	struct v4l2_mbus_framefmt fmt;

	if (format->type != video->type)
		return -EINVAL;

	mutex_lock(&video->mutex);

	/* Fill the bytesperline and sizeimage fields by converting to media bus
	 * format and back to pixel format.
	 */
	iss_video_pix_to_mbus(&format->fmt.pix, &fmt);
	iss_video_mbus_to_pix(video, &fmt, &format->fmt.pix);

	vfh->format = *format;

	mutex_unlock(&video->mutex);
	return 0;
}

static int
iss_video_try_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct iss_video *video = video_drvdata(file);
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	if (format->type != video->type)
		return -EINVAL;

	subdev = iss_video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	iss_video_pix_to_mbus(&format->fmt.pix, &fmt.format);

	fmt.pad = pad;
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);
	if (ret)
		return ret == -ENOIOCTLCMD ? -EINVAL : ret;

	iss_video_mbus_to_pix(video, &fmt.format, &format->fmt.pix);
	return 0;
}

static int
iss_video_cropcap(struct file *file, void *fh, struct v4l2_cropcap *cropcap)
{
	struct iss_video *video = video_drvdata(file);
	struct v4l2_subdev *subdev;
	int ret;

	subdev = iss_video_remote_subdev(video, NULL);
	if (subdev == NULL)
		return -EINVAL;

	mutex_lock(&video->mutex);
	ret = v4l2_subdev_call(subdev, video, cropcap, cropcap);
	mutex_unlock(&video->mutex);

	return ret == -ENOIOCTLCMD ? -EINVAL : ret;
}

static int
iss_video_get_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct iss_video *video = video_drvdata(file);
	struct v4l2_subdev_format format;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = iss_video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	/* Try the get crop operation first and fallback to get format if not
	 * implemented.
	 */
	ret = v4l2_subdev_call(subdev, video, g_crop, crop);
	if (ret != -ENOIOCTLCMD)
		return ret;

	format.pad = pad;
	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &format);
	if (ret < 0)
		return ret == -ENOIOCTLCMD ? -EINVAL : ret;

	crop->c.left = 0;
	crop->c.top = 0;
	crop->c.width = format.format.width;
	crop->c.height = format.format.height;

	return 0;
}

static int
iss_video_set_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct iss_video *video = video_drvdata(file);
	struct v4l2_subdev *subdev;
	int ret;

	subdev = iss_video_remote_subdev(video, NULL);
	if (subdev == NULL)
		return -EINVAL;

	mutex_lock(&video->mutex);
	ret = v4l2_subdev_call(subdev, video, s_crop, crop);
	mutex_unlock(&video->mutex);

	return ret == -ENOIOCTLCMD ? -EINVAL : ret;
}

static int
iss_video_get_param(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct iss_video_fh *vfh = to_iss_video_fh(fh);
	struct iss_video *video = video_drvdata(file);

	if (video->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ||
	    video->type != a->type)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.output.timeperframe = vfh->timeperframe;

	return 0;
}

static int
iss_video_set_param(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct iss_video_fh *vfh = to_iss_video_fh(fh);
	struct iss_video *video = video_drvdata(file);

	if (video->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ||
	    video->type != a->type)
		return -EINVAL;

	if (a->parm.output.timeperframe.denominator == 0)
		a->parm.output.timeperframe.denominator = 1;

	vfh->timeperframe = a->parm.output.timeperframe;

	return 0;
}

static int
iss_video_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *rb)
{
	struct iss_video_fh *vfh = to_iss_video_fh(fh);

	return vb2_reqbufs(&vfh->queue, rb);
}

static int
iss_video_querybuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct iss_video_fh *vfh = to_iss_video_fh(fh);

	return vb2_querybuf(&vfh->queue, b);
}

static int
iss_video_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct iss_video_fh *vfh = to_iss_video_fh(fh);

	return vb2_qbuf(&vfh->queue, b);
}

static int
iss_video_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct iss_video_fh *vfh = to_iss_video_fh(fh);

	return vb2_dqbuf(&vfh->queue, b, file->f_flags & O_NONBLOCK);
}

/*
 * Stream management
 *
 * Every ISS pipeline has a single input and a single output. The input can be
 * either a sensor or a video node. The output is always a video node.
 *
 * As every pipeline has an output video node, the ISS video objects at the
 * pipeline output stores the pipeline state. It tracks the streaming state of
 * both the input and output, as well as the availability of buffers.
 *
 * In sensor-to-memory mode, frames are always available at the pipeline input.
 * Starting the sensor usually requires I2C transfers and must be done in
 * interruptible context. The pipeline is started and stopped synchronously
 * to the stream on/off commands. All modules in the pipeline will get their
 * subdev set stream handler called. The module at the end of the pipeline must
 * delay starting the hardware until buffers are available at its output.
 *
 * In memory-to-memory mode, starting/stopping the stream requires
 * synchronization between the input and output. ISS modules can't be stopped
 * in the middle of a frame, and at least some of the modules seem to become
 * busy as soon as they're started, even if they don't receive a frame start
 * event. For that reason frames need to be processed in single-shot mode. The
 * driver needs to wait until a frame is completely processed and written to
 * memory before restarting the pipeline for the next frame. Pipelined
 * processing might be possible but requires more testing.
 *
 * Stream start must be delayed until buffers are available at both the input
 * and output. The pipeline must be started in the videobuf queue callback with
 * the buffers queue spinlock held. The modules subdev set stream operation must
 * not sleep.
 */
static int
iss_video_streamon(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct iss_video_fh *vfh = to_iss_video_fh(fh);
	struct iss_video *video = video_drvdata(file);
	enum iss_pipeline_state state;
	struct iss_pipeline *pipe;
	struct iss_video *far_end;
	unsigned long flags;
	int ret;
	if (type != video->type)
		return -EINVAL;

	mutex_lock(&video->stream_lock);

	if (video->streaming) {
		printk("%s:%d\n", __func__, __LINE__);
		mutex_unlock(&video->stream_lock);
		return -EBUSY;
	}

	/* Start streaming on the pipeline. No link touching an entity in the
	 * pipeline can be activated or deactivated once streaming is started.
	 */
	pipe = video->video.entity.pipe
	     ? to_iss_pipeline(&video->video.entity) : &video->pipe;
	pipe->external = NULL;
	pipe->external_rate = 0;
	pipe->external_bpp = 0;

	if (video->iss->pdata->set_constraints)
		video->iss->pdata->set_constraints(video->iss, true);
	ret = media_entity_pipeline_start(&video->video.entity, &pipe->pipe);
	if (ret < 0)
		goto err_media_entity_pipeline_start;
	/* Verify that the currently configured format matches the output of
	 * the connected subdev.
	 */
	ret = iss_video_check_format(video, vfh);
	if (ret < 0)
		goto err_iss_video_check_format;

	video->bpl_padding = ret;
	video->bpl_value = vfh->format.fmt.pix.bytesperline;

	/* Find the ISS video node connected at the far end of the pipeline and
	 * update the pipeline.
	 */
	far_end = iss_video_far_end(video);

	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		printk("video capture : %s:%d\n", __func__, __LINE__);
		state = ISS_PIPELINE_STREAM_OUTPUT | ISS_PIPELINE_IDLE_OUTPUT;
		pipe->input = far_end;
		pipe->output = video;
	} else {
		if (far_end == NULL) {
			ret = -EPIPE;
			goto err_iss_video_check_format;
		}

		state = ISS_PIPELINE_STREAM_INPUT | ISS_PIPELINE_IDLE_INPUT;
		pipe->input = video;
		pipe->output = far_end;
	}

	spin_lock_irqsave(&pipe->lock, flags);
	pipe->state &= ~ISS_PIPELINE_STREAM;
	pipe->state |= state;
	spin_unlock_irqrestore(&pipe->lock, flags);

	/* Set the maximum time per frame as the value requested by userspace.
	 * This is a soft limit that can be overridden if the hardware doesn't
	 * support the request limit.
	 */
	if (video->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		pipe->max_timeperframe = vfh->timeperframe;

	video->queue = &vfh->queue;
	INIT_LIST_HEAD(&video->dmaqueue);
	spin_lock_init(&video->qlock);
	atomic_set(&pipe->frame_number, -1);
	ret = vb2_streamon(&vfh->queue, type);
	if (ret < 0)
		goto err_iss_video_check_format;
	/* In sensor-to-memory mode, the stream can be started synchronously
	 * to the stream on command. In memory-to-memory mode, it will be
	 * started when buffers are queued on both the input and output.
	 */
	if (pipe->input == NULL) {
		unsigned long flags;
		ret = omap4iss_pipeline_set_stream(pipe,
					      ISS_PIPELINE_STREAM_CONTINUOUS);
		if (ret < 0)
			goto err_omap4iss_set_stream;
		spin_lock_irqsave(&video->qlock, flags);
		if (list_empty(&video->dmaqueue))
			video->dmaqueue_flags |= ISS_VIDEO_DMAQUEUE_UNDERRUN;
		spin_unlock_irqrestore(&video->qlock, flags);
	}

	if (ret < 0) {
err_omap4iss_set_stream:
		vb2_streamoff(&vfh->queue, type);
err_iss_video_check_format:
		media_entity_pipeline_stop(&video->video.entity);
err_media_entity_pipeline_start:
		if (video->iss->pdata->set_constraints)
			video->iss->pdata->set_constraints(video->iss, false);
		video->queue = NULL;
	}

	if (!ret)
		video->streaming = 1;

	mutex_unlock(&video->stream_lock);
	return ret;
}

static int
iss_video_streamoff(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct iss_video_fh *vfh = to_iss_video_fh(fh);
	struct iss_video *video = video_drvdata(file);
	struct iss_pipeline *pipe = to_iss_pipeline(&video->video.entity);
	enum iss_pipeline_state state;
	unsigned long flags;

	if (type != video->type)
		return -EINVAL;

	mutex_lock(&video->stream_lock);

	if (!vb2_is_streaming(&vfh->queue))
		goto done;

	/* Update the pipeline state. */
	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		state = ISS_PIPELINE_STREAM_OUTPUT
		      | ISS_PIPELINE_QUEUE_OUTPUT;
	else
		state = ISS_PIPELINE_STREAM_INPUT
		      | ISS_PIPELINE_QUEUE_INPUT;

	spin_lock_irqsave(&pipe->lock, flags);
	pipe->state &= ~state;
	spin_unlock_irqrestore(&pipe->lock, flags);

	/* Stop the stream. */
	omap4iss_pipeline_set_stream(pipe, ISS_PIPELINE_STREAM_STOPPED);
	vb2_streamoff(&vfh->queue, type);
	video->queue = NULL;
	video->streaming = 0;

	if (video->iss->pdata->set_constraints)
		video->iss->pdata->set_constraints(video->iss, false);
	media_entity_pipeline_stop(&video->video.entity);

done:
	mutex_unlock(&video->stream_lock);
	return 0;
}

static int
iss_video_enum_input(struct file *file, void *fh, struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;

	strlcpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int
iss_video_g_input(struct file *file, void *fh, unsigned int *input)
{
	*input = 0;

	return 0;
}

enum iss_size {
       ISS_SIZE_QVGA,
       ISS_SIZE_VGA,
       ISS_SIZE_720P,
       ISS_SIZE_1080P,
       ISS_SIZE_5MP,
       ISS_SIZE_LAST,
};
static const struct v4l2_frmsize_discrete iss_frmsizes[] = {
       {  320,  240 },
       {  640,  480 },
       { 1280,  720 },
       { 1920, 1080 },
       { 2592, 1944 },
};

static int iss_vidioc_enum_framesizes(struct file *filp,
		                void *priv, struct v4l2_frmsizeenum *frms)
{
	switch (frms->pixel_format) {
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_SBGGR8:
		       frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
                       frms->discrete.width = iss_frmsizes[frms->index].width;
                       frms->discrete.height = iss_frmsizes[frms->index].height;
			return 0;
		default: return -EINVAL;
	}
}

static int iss_vidioc_enum_frameintervals(struct file *file, void *priv,
		                            struct v4l2_frmivalenum *fe)
{
	fe->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fe->discrete.denominator = 30000;
	fe->discrete.numerator = 1001;
	printk("%s discrete %d/%d\n", __func__, fe->discrete.numerator,
			fe->discrete.denominator);
	return 0;
}

static int iss_vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
		                               struct v4l2_fmtdesc *f)
{
	char format_name[] = "4:2:2, packed, UYVY";

	strlcpy(f->description, format_name, sizeof(format_name));
	f->pixelformat = V4L2_PIX_FMT_UYVY;
	return 0;
}


static const struct v4l2_ioctl_ops iss_video_ioctl_ops = {
	.vidioc_querycap		= iss_video_querycap,
	.vidioc_g_fmt_vid_cap		= iss_video_get_format,
	.vidioc_s_fmt_vid_cap		= iss_video_set_format,
	.vidioc_try_fmt_vid_cap		= iss_video_try_format,
	.vidioc_g_fmt_vid_out		= iss_video_get_format,
	.vidioc_s_fmt_vid_out		= iss_video_set_format,
	.vidioc_try_fmt_vid_out		= iss_video_try_format,
	.vidioc_cropcap			= iss_video_cropcap,
	.vidioc_g_crop			= iss_video_get_crop,
	.vidioc_s_crop			= iss_video_set_crop,
	.vidioc_g_parm			= iss_video_get_param,
	.vidioc_s_parm			= iss_video_set_param,
	.vidioc_reqbufs			= iss_video_reqbufs,
	.vidioc_querybuf		= iss_video_querybuf,
	.vidioc_qbuf			= iss_video_qbuf,
	.vidioc_dqbuf			= iss_video_dqbuf,
	.vidioc_streamon		= iss_video_streamon,
	.vidioc_streamoff		= iss_video_streamoff,
	.vidioc_enum_input		= iss_video_enum_input,
	.vidioc_g_input			= iss_video_g_input,
	.vidioc_enum_framesizes		= iss_vidioc_enum_framesizes,
	.vidioc_enum_frameintervals	= iss_vidioc_enum_frameintervals,
	.vidioc_enum_fmt_vid_cap	= iss_vidioc_enum_fmt_vid_cap,
};

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */

static int iss_video_open(struct file *file)
{
	struct iss_video *video = video_drvdata(file);
	struct iss_video_fh *handle;
	struct vb2_queue *q;
	int ret = 0;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (handle == NULL)
		return -ENOMEM;

	v4l2_fh_init(&handle->vfh, &video->video);
	v4l2_fh_add(&handle->vfh);

	/* If this is the first user, initialise the pipeline. */
	if (omap4iss_get(video->iss) == NULL) {
		ret = -EBUSY;
		goto done;
	}

	ret = omap4iss_pipeline_pm_use(&video->video.entity, 1);
	if (ret < 0) {
		omap4iss_put(video->iss);
		goto done;
	}

	video->alloc_ctx = vb2_dma_contig_init_ctx(video->iss->dev);
	if (IS_ERR(video->alloc_ctx)) {
		ret = PTR_ERR(video->alloc_ctx);
		omap4iss_put(video->iss);
		goto done;
	}

	q = &handle->queue;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP;
	q->drv_priv = handle;
	q->ops = &iss_video_vb2ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct iss_buffer);

	ret = vb2_queue_init(q);
	if (ret) {
		omap4iss_put(video->iss);
		goto done;
	}

	memset(&handle->format, 0, sizeof(handle->format));
	handle->format.type = video->type;
	handle->timeperframe.denominator = 1;

	handle->video = video;
	file->private_data = &handle->vfh;

done:
	if (ret < 0) {
		v4l2_fh_del(&handle->vfh);
		kfree(handle);
	}

	return ret;
}

static int iss_video_release(struct file *file)
{
	struct iss_video *video = video_drvdata(file);
	struct v4l2_fh *vfh = file->private_data;
	struct iss_video_fh *handle = to_iss_video_fh(vfh);

	/* Disable streaming and free the buffers queue resources. */
	iss_video_streamoff(file, vfh, video->type);

	omap4iss_pipeline_pm_use(&video->video.entity, 0);

	/* Release the videobuf2 queue */
	vb2_queue_release(&handle->queue);

	/* Release the file handle. */
	v4l2_fh_del(vfh);
	kfree(handle);
	file->private_data = NULL;

	omap4iss_put(video->iss);

	return 0;
}

static unsigned int iss_video_poll(struct file *file, poll_table *wait)
{
	struct iss_video_fh *vfh = to_iss_video_fh(file->private_data);

	return vb2_poll(&vfh->queue, file, wait);
}

static int iss_video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct iss_video_fh *vfh = to_iss_video_fh(file->private_data);

	return vb2_mmap(&vfh->queue, vma);;
}

static struct v4l2_file_operations iss_video_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = iss_video_open,
	.release = iss_video_release,
	.poll = iss_video_poll,
	.mmap = iss_video_mmap,
};

/* -----------------------------------------------------------------------------
 * ISS video core
 */

static const struct iss_video_operations iss_video_dummy_ops = {
};

int omap4iss_video_init(struct iss_video *video, const char *name)
{
	const char *direction;
	int ret;

	switch (video->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		direction = "output";
		video->pad.flags = MEDIA_PAD_FL_SINK;
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		direction = "input";
		video->pad.flags = MEDIA_PAD_FL_SOURCE;
		break;

	default:
		return -EINVAL;
	}

	ret = media_entity_init(&video->video.entity, 1, &video->pad, 0);
	if (ret < 0)
		return ret;

	mutex_init(&video->mutex);
	atomic_set(&video->active, 0);

	spin_lock_init(&video->pipe.lock);
	mutex_init(&video->stream_lock);

	/* Initialize the video device. */
	if (video->ops == NULL)
		video->ops = &iss_video_dummy_ops;

	video->video.fops = &iss_video_fops;
	snprintf(video->video.name, sizeof(video->video.name),
		 "OMAP4 ISS %s %s", name, direction);
	video->video.vfl_type = VFL_TYPE_GRABBER;
	video->video.release = video_device_release_empty;
	video->video.ioctl_ops = &iss_video_ioctl_ops;
	video->pipe.stream_state = ISS_PIPELINE_STREAM_STOPPED;

	video_set_drvdata(&video->video, video);

	return 0;
}

void omap4iss_video_cleanup(struct iss_video *video)
{
	media_entity_cleanup(&video->video.entity);
	mutex_destroy(&video->stream_lock);
	mutex_destroy(&video->mutex);
}

int omap4iss_video_register(struct iss_video *video, struct v4l2_device *vdev)
{
	int ret;

	video->video.v4l2_dev = vdev;

	ret = video_register_device(&video->video, VFL_TYPE_GRABBER, -1);
	if (ret < 0)
		printk(KERN_ERR "%s: could not register video device (%d)\n",
			__func__, ret);

	return ret;
}

void omap4iss_video_unregister(struct iss_video *video)
{
	if (video_is_registered(&video->video))
		video_unregister_device(&video->video);
}
