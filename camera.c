#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>		/* getopt_long() */

#include <fcntl.h>		/* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#ifdef __ANDROID__
#include <android/log.h>
#endif

using namespace cv;
using namespace std;

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define XU_CTRL_NUM	3
#define DEBUG 0

#define D(format, ...) \
    if (DEBUG)         \
printf(format " %s %d", __VA_ARGS__, __FILE__, __LINE__)

#ifdef __ANDROID__
#define LOG_TAG "CAM"
#define logs(...) \
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#else
#define logs(format, ...) \
    do { \
        fprintf(stderr, "DEBUG %s:%d: " format "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
    } \
while(0)
#endif /* __ANDROID__ */


/* function forward decalration */
void tof_sensor_probe(struct sensor_device *);
static void tof_sensor_start(void);
static void tof_sensor_config(struct sensor_device*, struct config_request_buffer);
static void tof_sensor_capture(struct sensor_device *);
static void tof_sensor_stop(void);

void rgb_sensor_probe(struct sensor_device *);
static void rgb_sensor_start(void);
static void rgb_sensor_config(struct sensor_device*, struct config_request_buffer);
static void rgb_sensor_capture(struct sensor_device *);
static void rgb_sensor_stop(void);

static void errno_exit(const char *s);
static int xioctl(int fh, int request, void *arg);
static void open_device(void);
void deinit_cam_module(void);
static void dev_drv_match();
static int probe_devices();
static int open_device_with_dev_name(const char*);
static struct sensor_device* getSensor(const char* name);
static void init_device(const int);
void get_video(struct sensor_device*, const int);
struct sensor_device* getTOFSensor();
struct sensor_device* getRGBSensor();
void render_tof();


struct image_output_mode {
    int height;
    int width;
    int format;
    int channels;
};

struct sensor_info {
    const char *card;
    unsigned int vid;
    unsigned int pid;
};


struct module_info {
    struct sensor_info rgbSensor;
    struct sensor_info tofSensor;
};

struct module_info modules[] = {
    {
        {
            .card = "USB2.0 Camera",
            .vid  = 0,
            .pid  = 0 
        },
        {
            .card = "USB3.0 Camera",
            .vid  = 1,
            .pid  = 1 
        },
    },
    {
        {
            .card = "Camera1",
            .vid  = 2,
            .pid  = 2 
        },
        {
            .card = "Camera2",
            .vid  = 3,
            .pid  = 3
        },
    }
};

struct sensor_driver {
    const char *name;
    unsigned int n_buffers;
    struct buffer *buffers;
    void (*probe)(struct sensor_device *);
    void (*open)(void);
    void (*config)(struct sensor_device*, struct config_request_buffer);
    void (*start)(void);
    void (*capture)(struct sensor_device*);
    void (*stop)(void);
    void (*close)(void);
};

static struct sensor_driver tof_driver = {
    .name       = "TOF",
    .n_buffers  = 0,
    .buffers    = NULL,
    .probe      = tof_sensor_probe,
    .open       = NULL,
    .config     = tof_sensor_config,
    .start      = tof_sensor_start,
    .capture    = tof_sensor_capture,
    .stop       = tof_sensor_stop
};

static struct sensor_driver rgb_driver = {
    .name       = "RGB",
    .n_buffers  = 0,
    .buffers    = NULL,
    .probe      = rgb_sensor_probe,
    .open       = NULL,
    .config     = rgb_sensor_config,
    .start      = rgb_sensor_start,
    .capture    = rgb_sensor_capture,
    .stop       = rgb_sensor_stop,
};

struct sensor_device {
    const char *name;
    int id;
    void *frame_data;
    struct image_output_mode output_mode;
    struct sensor_info *sensor;
    struct sensor_driver *drv;
    void (*config)(struct sensor_device*, struct config_request_buffer);
    void (*start)(void);
    void (*capture)(struct sensor_device*);
    void (*stop)(void);
};

static struct sensor_device rgb_device = {
    .name = "RGB",
    .id = -1,
    .frame_data = NULL, 
};

static struct sensor_device tof_device = {
    .name = "TOF",
    .id = -1,
    .frame_data = NULL,
};

struct sensor_device_list {
    struct sensor_device *dev;
    struct sensor_device_list *next;
};

struct sensor_driver_list {
    struct sensor_driver *drv;
    struct sensor_driver_list *next;
};

static struct sensor_device_list *sensor_list = NULL;
static const struct sensor_driver_list *driver_list = NULL;


enum io_method {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
};

enum device_type {
    DEVICE_TOF,
    DEVICE_RGB,
    DEVICE_MAX,
};

enum error_no {
    err_no_error = 0x0,
    err_malloc_fail = 0x80000000,
    err_is_null,
    err_out_of_bound,
};

enum config_ctrl {
    config_set_format,
    config_get_format,
    config_max,

};

struct config_request_buffer {
    struct image_output_mode mode;
    enum config_ctrl        ctrl;
};

struct buffer {
    void            *start;
    size_t          length;
};

static char       *dev_name;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd_tof = -1;
static int              fd_rgb = -1;
static int              fd = -1;
//struct buffer           *buffers;
//static unsigned int     n_buffers;
static int              force_format = 1;
static IplImage*        out_frame = NULL;

static struct sensor_device_list* sensor_list_create(struct sensor_device *dev)
{
    struct sensor_device_list *ptr;
    ptr = (struct sensor_device_list*)malloc(sizeof(struct sensor_device_list));
    if (NULL == ptr)
        errno_exit("allocation fail");

    ptr->dev = dev;
    ptr->next = NULL;

    sensor_list = ptr;

    return ptr;
}

static int sensor_register_device(struct sensor_device *pdev)
{
    /* sanity check */
    if (NULL == pdev)
        errno_exit("sensor device NULL");

    if (NULL == sensor_list) {
        sensor_list_create(pdev);
        return (int)err_no_error;
    }

    struct sensor_device_list *ptr = NULL;
    struct sensor_device_list *curr = sensor_list;
    ptr = (struct sensor_device_list*)malloc(sizeof(struct sensor_device_list));
    
    if (NULL == ptr) {
        errno_exit("malloc fail");
    }

    ptr->dev = pdev;
    ptr->next = NULL;

    /* loop to the end of list */
    while (curr->next != NULL) {
        curr = curr->next;
    }

    /* Now we add the device sensor */
    curr->next = ptr;

    return (int)err_no_error;
}

static struct sensor_driver* driver_array[2] = {0};
int sensor_register_driver(struct sensor_driver *drv, const int index)
{
    if(index > 1)
        errno_exit("erro index out of bound");
    driver_array[index] = drv;
}


void rgb_sensor_probe(struct sensor_device *pdev)
{
    struct stat st;
    int fd;
    /* device id from 0 to 99, no more */
    dev_name = (char*)malloc(sizeof("/dev/videoxx"));
    memset(dev_name, 0x0, sizeof("/dev/videoxx"));

    printf("Probing RGB sensor\n");
    for (int i = 0; i < 10;i++) {
        sprintf(dev_name, "/dev/video%d", i);
        printf("dev_name is %s\n", dev_name);

        fd_rgb = open_device_with_dev_name(dev_name);
        if (-1 == fd_rgb)
            continue;

        struct v4l2_capability cap;

        if (-1 == xioctl(fd_rgb, VIDIOC_QUERYCAP, &cap)) {
            if (EINVAL == errno) {
                fprintf(stderr, "%s is no V4L2 device\n",
                        dev_name);
                exit(EXIT_FAILURE);
            } else {
                errno_exit("VIDIOC_QUERYCAP");
            }
        }

        printf("######cap.card is %s#######\n", cap.card);
        for (int i = 0; i < sizeof(modules)/sizeof(struct module_info); i++) {
            printf("#####matching %s#####\n", (char*)modules[i].rgbSensor.card);
            if (!strcmp((char*)cap.card, (char*)modules[i].rgbSensor.card)) {
                pdev->id = fd_rgb;
                printf("found cap.card is %s, id is %d\n", cap.card, pdev->id);
                return;
            }
        }
        /* if we reach here, this fd is not what we want, release it*/
        printf("RGB dev %s is not what we want, release it\n", dev_name);
        close(fd_rgb);
    } //end for

    free(dev_name);
    return;
}

void tof_sensor_probe(struct sensor_device *pdev)
{
    struct stat st;
    int fd;
    /* device id from 0 to 99, no more */
    dev_name = (char*)malloc(sizeof("/dev/videoxx"));
    memset(dev_name, 0x0, sizeof("/dev/videoxx"));

    printf("Probing TOF sensor\n");
    for (int i = 0; i < 10;i++) {
        sprintf(dev_name, "/dev/video%d", i);
        printf("dev_name is %s\n", dev_name);


        fd_tof = open_device_with_dev_name(dev_name);
        if (-1 == fd_tof)
            continue;

        struct v4l2_capability cap;

        if (-1 == xioctl(fd_tof, VIDIOC_QUERYCAP, &cap)) {
            if (EINVAL == errno) {
                fprintf(stderr, "%s is no V4L2 device\n",
                        dev_name);
                exit(EXIT_FAILURE);
            } else {
                errno_exit("VIDIOC_QUERYCAP");
            }
        }

        printf("######cap.card is %s#######\n", cap.card);
        for (int i = 0; i < sizeof(modules)/sizeof(struct module_info); i++) {
            printf("#####matching %s#####\n", (char*)modules[i].tofSensor.card);
            if (!strcmp((char*)cap.card, (char*)modules[i].tofSensor.card)) {
                pdev->id = fd_tof;
                printf("found cap.card is %s, id is %d\n", cap.card, pdev->id);
                return;
            }
        }
        printf("TOF dev %s is not what we want, release it\n", dev_name);
        close(fd_tof);
    } //end for
    return;
}

static void errno_exit(const char *s)
{
    logs("%s error %d, %s\n", s, errno, strerror(errno));
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
    int r;

    //logs("Sending UVC ioctl %d\n", request);
    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);

    return r;
}

void setMirror(int, int);
static void process_image(const void *p, int size, void *out_data)
{
    //printf("processed image size is %d\n", size);
    memcpy(out_data, p, size);
}

static int read_frame(struct sensor_device* pdev, const int fd)
{
    struct v4l2_buffer buf;
    unsigned int i;

    switch (io) {
        case IO_METHOD_READ:
            if (-1 == read(fd, pdev->drv->buffers[0].start, pdev->drv->buffers[0].length)) {
                switch (errno) {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("read");
                }
            }

            process_image(pdev->drv->buffers[0].start, pdev->drv->buffers[0].length, pdev->frame_data);
            break;

        case IO_METHOD_MMAP:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                switch (errno) {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            assert(buf.index < pdev->drv->n_buffers);

            process_image(pdev->drv->buffers[buf.index].start, buf.bytesused, pdev->frame_data);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF");
            break;

        case IO_METHOD_USERPTR:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                switch (errno) {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            for (i = 0; i < pdev->drv->n_buffers; ++i)
                if (buf.m.userptr == (unsigned long)pdev->drv->buffers[i].start
                        && buf.length == pdev->drv->buffers[i].length)
                    break;

            assert(i < pdev->drv->n_buffers);

            process_image((void *)buf.m.userptr, buf.bytesused, pdev->frame_data);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF");
            break;
    }

    return 1;
}

static int frame_count = 0;
void usb_init(void);
void dummy_get_video()
{
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(fd_tof, &fds);

    /* Timeout. */
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    r = select(fd_tof + 1, &fds, NULL, NULL, &tv);

    if (-1 == r) {
        if (EINTR == errno)
            printf("errorno is EINTR");
        errno_exit("select");
    }

    if (0 == r) {
        fprintf(stderr, "select timeout\n");
        //exit(EXIT_FAILURE);
    }

}

void get_video(struct sensor_device *pdev, const int fd)
{
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    /* Timeout. */
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    r = select(fd+ 1, &fds, NULL, NULL, &tv);

    if (-1 == r) {
        if (EINTR == errno)
            printf("errorno is EINTR");
        errno_exit("select");
    }

    if (0 == r) {
        fprintf(stderr, "select timeout\n");
//        exit(EXIT_FAILURE);
    }

    read_frame(pdev, fd);

    frame_count++;
    /* EAGAIN - continue select loop. */
}

static void stop_capturing(const int fd)
{
    enum v4l2_buf_type type;

    switch (io) {
        case IO_METHOD_READ:
            /* Nothing to do. */
            break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                errno_exit("VIDIOC_STREAMOFF");
            break;
    }
}

static void start_capturing(struct sensor_device *pdev, const int fd)
{
    unsigned int i;
    enum v4l2_buf_type type;

    printf("Start capturing fd %d\n", fd);
    switch (io) {
        case IO_METHOD_READ:
            /* Nothing to do. */
            break;

        case IO_METHOD_MMAP:
            /* We also init frame image here*/

            for (i = 0; i < pdev->drv->n_buffers; ++i) {
                struct v4l2_buffer buf;

                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;

                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
                    printf("can not do ioctl\n");
                    errno_exit("VIDIOC_QBUF");
                }
            }

            logs("before type\n");
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                errno_exit("VIDIOC_STREAMON");

            logs("after stream on\n");
            break;

        case IO_METHOD_USERPTR:
            for (i = 0; i < pdev->drv->n_buffers; ++i) {
                struct v4l2_buffer buf;

                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_USERPTR;
                buf.index = i;
                buf.m.userptr = (unsigned long)pdev->drv->buffers[i].start;
                buf.length = pdev->drv->buffers[i].length;

                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            }
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                errno_exit("VIDIOC_STREAMON");
            break;
    }
}


static void close_device(void)
{
    if (-1 != fd_tof) {
        if (-1 == close(fd_tof))
            errno_exit("close");
        fd_tof = -1;
    }

    if (-1 != fd_rgb) {
        if (-1 == close(fd_rgb))
            errno_exit("close");
        fd_rgb = -1;
    }

}

static int get_device_fd(void)
{
    return 0;

}

static enum device_type check_device_type(void)
{
    int fd = -1;
    struct v4l2_capability cap;

    fd = get_device_fd();

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                    dev_name);
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

    for (int i = 0; i < sizeof(modules)/sizeof(struct module_info); i++) {
        const char *tofname = modules[i].tofSensor.card;
        const char *rgbname = modules[i].rgbSensor.card;
        if (!strcmp((char*)cap.card, (char*)tofname)) {
            printf("found tof cap.card is %s\n", cap.card);
            return DEVICE_TOF;
        } else if (!strcmp((char*)cap.card, (char*)rgbname)) {
            printf("found rgb cap.card is %s\n", cap.card);
            return DEVICE_RGB;
        }
    }
}

static int open_device_with_dev_name(const char *dev_name)
{
    struct stat st;
    int fd;

    if (-1 == stat(dev_name, &st)) {
        logs("Cannot identify %s", dev_name);
        fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        return -1;
        exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode)) {
        logs("%s is no device!", dev_name);
        fprintf(stderr, "%s is no device\n", dev_name);
        exit(EXIT_FAILURE);
    }

    fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd) {
        logs("Cannot open %s", dev_name);
        fprintf(stderr, "Cannot open '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        goto exit;
        exit(EXIT_FAILURE);
    }

    printf("successfully opened device %d\n", fd);

    struct v4l2_capability cap;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                    dev_name);
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

exit:
    return fd;

}

static void open_device(void)
{
    struct stat st;

    logs("In Open device, dev is %s", dev_name);
    if (-1 == stat(dev_name, &st)) {
        logs("Cannot identify %s", dev_name);
        fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode)) {
        logs("%s is no device!", dev_name);
        fprintf(stderr, "%s is no device\n", dev_name);
        exit(EXIT_FAILURE);
    }

    fd_tof = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd_tof) {
        logs("Cannot open %s", dev_name);
        fprintf(stderr, "Cannot open '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
    printf("successfully opened device %d\n", fd_tof);
}


static void uninit_device(struct sensor_device *pdev)
{
    unsigned int i;

    switch (io) {
        case IO_METHOD_READ:
            free(pdev->drv->buffers[0].start);
            break;

        case IO_METHOD_MMAP:
            for (i = 0; i < pdev->drv->n_buffers; ++i)
                if (-1 == munmap(pdev->drv->buffers[i].start, pdev->drv->buffers[i].length))
                    errno_exit("munmap");
            break;

        case IO_METHOD_USERPTR:
            for (i = 0; i < pdev->drv->n_buffers; ++i)
                free(pdev->drv->buffers[i].start);
            break;
    }

    free(pdev->drv->buffers);
}

static void init_read(struct sensor_device* pdev, unsigned int buffer_size)
{

    pdev->drv->buffers = (struct buffer *)calloc(1, sizeof(*(pdev->drv->buffers)));

    if (!pdev->drv->buffers) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    pdev->drv->buffers[0].length = buffer_size;
    pdev->drv->buffers[0].start = malloc(buffer_size);

    if (!pdev->drv->buffers[0].start) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }
}

static void un_init_mmap(const int fd)
{
    struct v4l2_requestbuffers req;
    CLEAR(req);

    req.count = 0;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    /* free uvc driver userspace buffers */
    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s does not support "
                    "memory mapping\n", dev_name);
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

}

static void init_mmap(struct sensor_device *pdev, const int fd)
{
    struct v4l2_requestbuffers req;

    printf("init mmap\n");
    CLEAR(req);

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s does not support "
                    "memory mapping\n", dev_name);
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2) {
        fprintf(stderr, "Insufficient buffer memory on %s\n",
                dev_name);
        exit(EXIT_FAILURE);
    }

    pdev->drv->buffers = (struct buffer *)calloc(req.count, sizeof(*(pdev->drv->buffers)));

    if (!pdev->drv->buffers) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (pdev->drv->n_buffers = 0; pdev->drv->n_buffers < req.count; ++pdev->drv->n_buffers) {

        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory  = V4L2_MEMORY_MMAP;
        buf.index   = pdev->drv->n_buffers;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
            errno_exit("VIDIOC_QUERYBUF");

        pdev->drv->buffers[pdev->drv->n_buffers].length = buf.length;
        pdev->drv->buffers[pdev->drv->n_buffers].start =
            mmap(NULL /* start anywhere */,
                    buf.length,
                    PROT_READ | PROT_WRITE /* required */,
                    MAP_SHARED /* recommended */,
                    fd, buf.m.offset);

        if (MAP_FAILED == pdev->drv->buffers[pdev->drv->n_buffers].start)
            errno_exit("mmap");
    }
}

static void init_userp(struct sensor_device* pdev, unsigned int buffer_size)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count  = 4;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s does not support "
                    "user pointer i/o\n", dev_name);
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    pdev->drv->buffers = (struct buffer *)calloc(4, sizeof(*(pdev->drv->buffers)));

    if (!pdev->drv->buffers) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (pdev->drv->n_buffers = 0; pdev->drv->n_buffers < 4; ++pdev->drv->n_buffers) {
        pdev->drv->buffers[pdev->drv->n_buffers].length = buffer_size;
        pdev->drv->buffers[pdev->drv->n_buffers].start = malloc(buffer_size);

        if (!pdev->drv->buffers[pdev->drv->n_buffers].start) {
            fprintf(stderr, "Out of memory\n");
            exit(EXIT_FAILURE);
        }
    }
}


static void xu_query(__u8 unit, __u8 sel, __u8 query, __u16 sz, __u8 *buf)
{
    struct uvc_xu_control_query uvc;

    uvc.unit = unit;
    uvc.selector = sel;
    uvc.query = query;
    uvc.size = sz;
    uvc.data = buf;

    if (-1 == xioctl(getTOFSensor()->id, UVCIOC_CTRL_QUERY, &uvc)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s UVCIOC_CTRL_QUERY failed.\n",
                    dev_name);
            exit(EXIT_FAILURE);
        } else {
            errno_exit("UVCIOC_CTRL_QUERY");
        }
    }
}

void getLenInfoTest(int sel)
{
    __u8 buf[3];
    unsigned short len;

    xu_query(XU_CTRL_NUM, sel, UVC_GET_LEN, 2, buf);
    len = (unsigned short )(buf[0]) + (unsigned short )(buf[1] << 8);
    printf("sel %d lenth is %d", sel, len);

    xu_query(XU_CTRL_NUM, sel, UVC_GET_INFO, 1, buf);
    printf("   info is 0b%x\n\n", buf[0]);
}

void showSelInfo()
{
    getLenInfoTest(1);
    getLenInfoTest(2);
    getLenInfoTest(3);
    getLenInfoTest(4);
    getLenInfoTest(5);
    /*
       getLenInfoTest(6);
       getLenInfoTest(7);
       getLenInfoTest(8);
       getLenInfoTest(9);
       getLenInfoTest(10);
     */
}

void usb_init(void)
{
    /* configure sensor here */
}

int set_video_format(struct image_output_mode mode, const int fd)
{
    struct v4l2_format fmt;
    int ret;

    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    logs("setting fd %d\n", fd);
    if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
        errno_exit("VIDIOC_G_FMT");

    fmt.fmt.pix.width       = mode.width;
    fmt.fmt.pix.height      = mode.height;

    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) {
        switch (errno) {
            case EBUSY:
                printf("device busy, please stop stream before retry\n");
                return errno;
            default:
                errno_exit("VIDIOC_S_FMT");
                break;
        }
    }

    return 0;

}

static int config_video(struct sensor_device* pdev, struct config_request_buffer config)
{
    int ret, fd;
    struct image_output_mode* mode = &config.mode;


    if (!pdev) {
        printf("device is NULL, can not config\n");
    }
    if (pdev->frame_data) {
        printf("freeing pre-allocate frame data\n");
        free(pdev->frame_data);
    }
    fd = pdev->id;
    pdev->frame_data = malloc(mode->height * mode->width * mode->channels);


    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
        errno_exit("VIDIOC_STREAMOFF");


    printf("about to config fd %d\n", fd);
    switch(config.ctrl) {
        case config_set_format:
        {
            ret = set_video_format(config.mode, fd);
            if (EBUSY == ret ) {
                printf("STREAM OFF\n");
#if 0
                stop_capturing(fd); /* STREMOFF */
                uninit_device();    /* munmap */
                un_init_mmap(fd);
#endif
                set_video_format(config.mode, fd);
            }
        }
            break;
        case config_get_format:
            break;
        default:
            break;
    }

    init_mmap(pdev, fd);
    printf("STREAM ON\n");
    start_capturing(pdev, fd);
    /*
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");
*/
    return ret;
}

static void tof_sensor_config(struct sensor_device* pdev, struct config_request_buffer config)
{
    
    memcpy((void*)&pdev->output_mode, &config, sizeof(config));
    printf("int tof sensor config , fd is %d\n", pdev->id);
    if (0 != strcmp(pdev->name, "TOF")) {
        printf("Incorrect device name %s, expecting TOF\n", pdev->name);
    }
    config_video(pdev, config);
    /* dummy workaround for configuration to take effetc, TOF only*/
    sleep(3);
    usb_init();
}

static void rgb_sensor_config(struct sensor_device* pdev, struct config_request_buffer config)
{
    memcpy((void*)&pdev->output_mode, &config, sizeof(config));
        printf("height %d\n", pdev->output_mode.height);
        printf("wiegh %d\n", pdev->output_mode.width);
        printf("channel%d\n", pdev->output_mode.channels);
    if (0 != strcmp(pdev->name, "RGB")) {
        printf("Incorrect device name %s, expecting RGB\n", pdev->name);
    }

    config_video(pdev, config);
}

static void tof_sensor_start(void)
{
    init_device(getTOFSensor()->id);
    start_capturing(getTOFSensor(), getTOFSensor()->id);
    printf("tof fd is %d\n", getTOFSensor()->id);
    sleep(3);
//    showSelInfo();
    usb_init();
}

static void tof_sensor_capture(struct sensor_device *pdev)
{
    get_video(pdev, pdev->id);
}

static void tof_sensor_stop()
{
}

static void rgb_sensor_stop()
{
}

static void rgb_sensor_capture(struct sensor_device *pdev)
{
    get_video(pdev, pdev->id);
}

static void rgb_sensor_start()
{
    int rgb_fd = getRGBSensor()->id;
    printf("rgb fd is %d\n", rgb_fd);
    init_device(rgb_fd);
    start_capturing(getRGBSensor(), rgb_fd);
}

static void init_device(const int __fd)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    struct uvc_xu_control_query uvc;
    unsigned int min;
    __u8 buf[64];

    bzero(&uvc, sizeof(uvc));
    bzero(buf, sizeof(buf));

    if (-1 == xioctl(__fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                    dev_name);
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "%s is no video capture device\n",
                dev_name);
        exit(EXIT_FAILURE);
    }
    fprintf(stderr, "cap.driver is %s\n", cap.driver);
    fprintf(stderr, "cap.card is %s\n", cap.card);

#if 1
    switch (io) {
        case IO_METHOD_READ:
            if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
                fprintf(stderr, "%s does not support read i/o\n",
                        dev_name);
                exit(EXIT_FAILURE);
            }
            break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                fprintf(stderr, "%s does not support streaming i/o\n",
                        dev_name);
                exit(EXIT_FAILURE);
            }
            break;
    }


    /* Select video input, video standard and tune here. */


    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(__fd, VIDIOC_CROPCAP, &cropcap)) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(__fd, VIDIOC_S_CROP, &crop)) {
            switch (errno) {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                    break;
            }
        }
    } else {
        /* Errors ignored. */
    }


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (force_format) {
#if 1
        fmt.fmt.pix.width       = 640;
        fmt.fmt.pix.height      = 240;
#else
        fmt.fmt.pix.width       = 320;
        fmt.fmt.pix.height      = 120;
#endif
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field       = V4L2_FIELD_ANY;

        printf("fd to configure is %d\n", __fd);
        if (-1 == xioctl(__fd, VIDIOC_S_FMT, &fmt))
            errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    } else {
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(__fd, VIDIOC_G_FMT, &fmt))
            errno_exit("VIDIOC_G_FMT");
    }

    fmt.fmt.pix.width       = 640;
    fmt.fmt.pix.height      = 240;
    if (-1 == xioctl(__fd, VIDIOC_S_FMT, &fmt))
        errno_exit("VIDIOC_S_FMT");
    if (-1 == xioctl(__fd, VIDIOC_G_FMT, &fmt))
        errno_exit("VIDIOC_G_FMT");
    printf("height is %d and width is %d\n", fmt.fmt.pix.height, fmt.fmt.pix.width);
    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

#if 0
    switch (io) {
        case IO_METHOD_READ:
            init_read(fmt.fmt.pix.sizeimage);
            break;

        case IO_METHOD_MMAP:
            init_mmap(__fd);
            break;

        case IO_METHOD_USERPTR:
            init_userp(fmt.fmt.pix.sizeimage);
            break;
    }
#endif
#endif
}

void videoChmod()
{
    char mode[] = "0777";
    char buf[100] = "/dev/video0";
    int i;
    i = strtol(mode, 0, 8);
    if (chmod (buf,i) < 0)
    {
        logs("Error in chmod(%s, %s) - %d (%s)\n",
                buf, mode, errno, strerror(errno));
        exit(1);
    }
    logs("chmod /dev/video0 successfully");
}

void initRGB()
{

}

void register_devices()
{
    sensor_register_device(&rgb_device);
    sensor_register_device(&tof_device);
}

void register_drivers()
{
    sensor_register_driver(&tof_driver, 0);
    sensor_register_driver(&rgb_driver, 1);
}


void get_depth_video(void **out, int *size)
{
    struct sensor_device* pdev = getTOFSensor();

    getTOFSensor()->capture(getTOFSensor());
    *out = getTOFSensor()->frame_data;
    *size = pdev->output_mode.height * 
            pdev->output_mode.width *
            pdev->output_mode.channels;
}

void render_tof()
{
    int i = 0;
//    getTOFSensor()->start();
    Mat matImageDual(240, 640, CV_16UC1, Scalar(0));
     
    struct sensor_device* pdev = getTOFSensor();


    for (i = 0; i < 1; i++) {
        getTOFSensor()->capture(getTOFSensor());
        memcpy(matImageDual.data, getTOFSensor()->frame_data, 240*640*2);

        printf("%d\n", __LINE__);
        Mat matConfidence(240, 320, CV_16SC1, Scalar(0)), matPhase(240, 320, CV_16SC1, Scalar(0));
        for (int i = 0; i < matImageDual.rows; i++) {
            for (int j = 0; j < matImageDual.cols/16; j++) {
                for (int k = 0; k < 8; k++) {
                    int16_t conf_data = matImageDual.at<int16_t>(i,16*j+k);
                    int16_t phase_data = matImageDual.at<int16_t>(i,16*j+k+8);

                    matConfidence.at<int16_t>(i,8*j+k) = (int16_t)((conf_data & 0x0FFF));
                    matPhase.at<int16_t>(i,8*j+k) = (int16_t)((phase_data & 0x0FFF));
                }
            }
        }
        const int MODULELATION_FREQUENCY = 24;
        const float FLOAT_LIGHT_SPEED = 3e11; // constant light speed 3 * 10e11 in mm
        const float FLOAT_MODULATION_FREQUENCY = MODULELATION_FREQUENCY * 1000000;
        // modulation frequency about 24 MHz = 24 * 10e6
        const float INT_RANGE = FLOAT_LIGHT_SPEED / (2 * FLOAT_MODULATION_FREQUENCY); // The maximum distance

        printf("%d\n", __LINE__);
        Mat matPhaseDistance(480/2, 640/2, CV_16UC1, Scalar(0));
        matPhase.convertTo(matPhase, CV_16UC1);
        matPhaseDistance = matPhase * INT_RANGE/4096;
        Mat matPhase8U(480/2, 640/2, CV_8UC1, Scalar(0));
        printf("%d\n", __LINE__);
        for(int j=0; j<480/2; j++)
            for(int i=0; i<640/2; i++)
                matPhase8U.at<unsigned char>(j, i) = matPhaseDistance.at<unsigned short>(j, i) % 256;

        printf("%d\n", __LINE__);
        imshow("hello", matPhase8U);
        cvWaitKey(1);
    }
}

void get_image_video(void** out, int *size)
{
    struct sensor_device* pdev = getRGBSensor();
    pdev->capture(pdev);
    *out = (char*)pdev->frame_data;
    *size = pdev->output_mode.height * 
            pdev->output_mode.width *
            pdev->output_mode.channels;
}


void demo_rgb()
{
    int i;
    struct sensor_device* pdev = getRGBSensor();
    // getRGBSensor()->start();
   
    i = 0;
    printf("%d\n", __LINE__);
    //Mat matImageDualRGB(240*2, 640, CV_16UC1, Scalar(0));
    while (i < 1) {
        pdev->capture(pdev);

        static IplImage*        out_frame = NULL;
        out_frame = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 2);

        if (NULL == out_frame) {
            printf("out frame is null\n");
            return;
        }

        out_frame->imageData = (char*)pdev->frame_data;
    
        Mat matImageDualRGB(out_frame);

        printf("%d\n", __LINE__);
        Mat temp(480, 640, CV_8UC3);
        cvtColor(matImageDualRGB, temp, CV_YUV2BGRA_YUYV );
        imshow("fuck", temp);

        printf("frame %d\n", i);
        cvWaitKey(1);
        i++;

        cvReleaseImage(&out_frame);
    }
}

void cam_module_init()
{
    static int inited = 0;
    if (inited) {
        fprintf(stdout, "cam module already inited\n");
        return;
    }
    register_devices();
    register_drivers();
    dev_drv_match();
    probe_devices();

 
    getRGBSensor()->start();
    getTOFSensor()->start();
 


       struct config_request_buffer buf;
    CLEAR(buf);
    buf.ctrl = config_set_format;
    buf.mode.height = 480;
    buf.mode.width = 640;
    buf.mode.channels = 2;

    printf("before conifg\n");
    getRGBSensor()->config(getRGBSensor(), buf);

#if 1
    buf.mode.height = 240;
    buf.mode.width = 640;

    getTOFSensor()->config(getTOFSensor(), buf);
#endif
printf("fuck you\n");
    inited = 1;
return;

    int i;

    for (i = 0; i < 1000/5; i++) {
        render_tof();
        demo_rgb();
        printf("i is %d\n", i);
    }
return;

    deinit_cam_module();
} 

struct sensor_device* getRGBSensor()
{
    return getSensor("RGB");
}

struct sensor_device* getTOFSensor()
{
    return getSensor("TOF");
}

static struct sensor_device* getSensor(const char* name)
{
    struct sensor_device_list *ptr;

    for (ptr = sensor_list; ptr != NULL; ptr = ptr->next) {
        if (0 == strcmp(ptr->dev->name, name)) {
            /* check if we have a match */
            if (NULL != ptr->dev->drv ) {
                return ptr->dev;
            }
        }
    }
    logs("can not find %s sensor\n", name);
    return NULL;
}


static void dev_drv_match()
{
    struct sensor_device_list *ptr;
    for (ptr = sensor_list; ptr != NULL; ptr = ptr->next) {
        if (0 == strcmp(ptr->dev->name, rgb_driver.name)) {
            ptr->dev->drv = &rgb_driver;
            ptr->dev->start = rgb_driver.start;
            ptr->dev->capture = rgb_driver.capture;
            ptr->dev->stop = rgb_driver.stop;
            ptr->dev->config = rgb_driver.config;
            logs("matched RGB sensor");
        } else if (0 == strcmp(ptr->dev->name, tof_driver.name)) {
            ptr->dev->drv = &tof_driver;
            ptr->dev->start = tof_driver.start;
            ptr->dev->capture = tof_driver.capture;
            ptr->dev->stop = tof_driver.stop;
            ptr->dev->config = tof_driver.config;
            logs("matched TOF sensor");
        }
    }
}

static int probe_devices()
{
    struct sensor_device_list *ptr;
    for (ptr = sensor_list; ptr != NULL; ptr = ptr->next) {
        if (NULL != ptr->dev->drv) {
            ptr->dev->drv->probe(ptr->dev);
        }
    }
}

void deinit_cam_module()
{
    printf("Calling deinit cam module\n");
    static int deinited = 0;
    if (deinited)
        return;
    struct sensor_device_list *ptr;
    while (sensor_list != NULL){
        ptr = sensor_list;
        sensor_list = sensor_list->next;
        free(ptr);
    }
    close_device();
    deinited = 1;
}

/*! @} */
