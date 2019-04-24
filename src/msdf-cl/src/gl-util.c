#include <stdlib.h>
#include <stdio.h> // printf

#define CL_TARGET_OPENCL_VERSION 220
#include <CL/cl.h>

#include "msdf.h"
#include "msdfgen-ext.h"
#include "msdfgen.h"
#include <Contour.h>
#include <Shape.h>
#include <edge-coloring.h>
#include <edge-segments.h>
#include <import-font.h>
#include <msdf.h>

#define CHECK(e)                                           \
    do {                                                   \
        if (e != CL_SUCCESS) {                             \
            printf("Error on line %i: %i \n",__LINE__, e); \
            exit(e);                                       \
        }                                                  \
    } while (0);

typedef struct segment {
    cl_int color;
    cl_int npoints;
    cl_float2 points[];
} segment;

typedef struct contour {
    cl_int winding;
    cl_int nsegments;
    struct segment segments[];
} contour;

typedef struct glyph {
    cl_int ncontours;
    struct contour contours[];
} glyph;

typedef struct multi_distance {
    cl_float2 r;
    cl_float2 g;
    cl_float2 b;
} multi_distance;


char* readSource(const char *sourceFilename) {

   FILE *fp;
   int err;
   int size;

   char *source;

   fp = fopen(sourceFilename, "rb");
   if(fp == NULL) {
      printf("Could not open kernel file: %s\n", sourceFilename);
      exit(-1);
   }

   err = fseek(fp, 0, SEEK_END);
   if(err != 0) {
      printf("Error seeking to end of file\n");
      exit(-1);
   }

   size = ftell(fp);
   if(size < 0) {
      printf("Error getting file position\n");
      exit(-1);
   }

   err = fseek(fp, 0, SEEK_SET);
   if(err != 0) {
      printf("Error seeking to start of file\n");
      exit(-1);
   }

   source = (char *)malloc(size+1);
   if(source == NULL) {
      printf("Error allocating %d bytes for the program source\n", size+1);
      exit(-1);
   }

   err = fread(source, 1, size, fp);
   if(err != size) {
      printf("only read %d bytes\n", err);
      exit(0);
   }

   source[size] = '\0';

   return source;
}

cl_int err;
cl_platform_id platform;
cl_device_id device;
cl_context ctx;
cl_command_queue queue;
cl_program program;
cl_kernel msdf_kernel;

static inline cl_float2 Point2_to_vec2(msdfgen::Point2 p) { return {(cl_float)p.x, (cl_float)p.y}; }


int main() {

    err = clGetPlatformIDs(1, &platform, NULL);
    err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);
    ctx = clCreateContext(0, 1, &device, NULL, NULL, &err);
    queue = clCreateCommandQueueWithProperties(ctx, device, 0, &err);
    char *kernelSrc = readSource("kernel.cl");
    program = clCreateProgramWithSource(ctx, 1, (const char **)&kernelSrc, NULL, &err);
    free(kernelSrc);

    err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
    if (err != CL_SUCCESS) {
        printf("building program failed\n");
        if (err == CL_BUILD_PROGRAM_FAILURE) {
            size_t logSize;
            clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, 0,
                                  NULL, &logSize);
            char *log = (char *)malloc(logSize);
            clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG,
                                  logSize, log, NULL);
            printf("%s\n", log);
            free(log);
        }
        exit(-1);
    }
    msdf_kernel = clCreateKernel(program, "msdf", &err);


    msdfgen::Shape shape;
    msdf_font_handle f =
        msdf_load_font("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf");
    msdfgen::FontHandle *font = (msdfgen::FontHandle *)f->__handle;

    // msdfgen::loadGlyph(shape, font, '1');
    msdfgen::loadGlyph(shape, font, 0x00e4);
    // msdfgen::loadGlyph(shape, font, '#');
    // msdfgen::loadGlyph(shape, font, '0');
    // msdfgen::loadGlyph(shape, font, ' ');

    shape.normalize();
    edgeColoringSimple(shape, 3.0);

    size_t input_size = sizeof(struct glyph);
    for (msdfgen::Contour &c : shape.contours) {
        input_size += sizeof(contour);
        for (msdfgen::EdgeHolder &e : c.edges) {
            input_size += sizeof(segment);
            if (dynamic_cast<msdfgen::LinearSegment *>(e.edgeSegment))
                input_size += 2 * sizeof(cl_float2);
            if (dynamic_cast<msdfgen::QuadraticSegment *>(e.edgeSegment))
                input_size += 3 * sizeof(cl_float2);
            if (dynamic_cast<msdfgen::CubicSegment *>(e.edgeSegment))
                input_size += 4 * sizeof(cl_float2);
        }
    }
    void *input_buffer = malloc(input_size);

    struct glyph *glyph_data = (struct glyph *)input_buffer;
    {
        glyph_data->ncontours = shape.contours.size();
        contour *c = glyph_data->contours;
        for (msdfgen::Contour &_c : shape.contours) {
            c->nsegments = _c.edges.size();
            c->winding = _c.winding();
            segment *s = c->segments;
            for (msdfgen::EdgeHolder &_e : _c.edges) {
                s->color = _e->color;
                if (auto p = dynamic_cast<msdfgen::LinearSegment *>(_e.edgeSegment)) {
                    s->npoints = 2;
                    s->points[0] = Point2_to_vec2(p->p[0]);
                    s->points[1] = Point2_to_vec2(p->p[1]);
                } else if (auto p = dynamic_cast<msdfgen::QuadraticSegment *>(_e.edgeSegment)) {
                    s->npoints = 3;
                    s->points[0] = Point2_to_vec2(p->p[0]);
                    s->points[1] = Point2_to_vec2(p->p[1]);
                    s->points[2] = Point2_to_vec2(p->p[2]);
                } else if (auto p = dynamic_cast<msdfgen::CubicSegment *>(_e.edgeSegment)) {
                    s->npoints = 4;
                    s->points[0] = Point2_to_vec2(p->p[0]);
                    s->points[1] = Point2_to_vec2(p->p[1]);
                    s->points[2] = Point2_to_vec2(p->p[2]);
                    s->points[3] = Point2_to_vec2(p->p[3]);
                }
                /* Move s to the beginning of the next segment */
                s = (segment *)(((cl_float2 *)(s + 1)) + s->npoints);
            }
            /* s already points to the following contour in the list */
            c = (contour *)s;
        }
    }

    cl_float2 scale = {1.0, 1.0};
    cl_float2 translate = {0.0, 0.0};
    cl_float range = 4.0;

    float width = font->face->glyph->metrics.width / 64.0;
    float height = font->face->glyph->metrics.height / 64.0;
    size_t w = ceil((width + range) * scale.x);
    size_t h = ceil((height + range) * scale.x);



    cl_mem glyph_data_buf = clCreateBuffer(ctx, CL_MEM_READ_ONLY, input_size, NULL, &err);


    cl_image_format img_fmt = {.image_channel_order = CL_RGBA, 
                               .image_channel_data_type = CL_FLOAT};

    cl_image_desc img_dsc = {.image_type = CL_MEM_OBJECT_IMAGE2D,
                             .image_width = w,
                             .image_height = h};

    cl_mem output_buf =clCreateImage(ctx, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR | CL_MEM_HOST_READ_ONLY,
                                     &img_fmt, &img_dsc, NULL, &err);
    CHECK(err);
    
    clSetKernelArg(msdf_kernel, 0, sizeof(cl_mem), &glyph_data_buf);
    clSetKernelArg(msdf_kernel, 1, sizeof(cl_mem), &output_buf);
    clSetKernelArg(msdf_kernel, 2, sizeof(cl_float2), &scale);
    clSetKernelArg(msdf_kernel, 3, sizeof(cl_float2), &translate);
    clSetKernelArg(msdf_kernel, 4, sizeof(cl_float), &range);
    
    size_t global_work_size[] = {w, h};
    size_t local_work_size[] = {4, 4};
    err = clEnqueueNDRangeKernel(queue, msdf_kernel, 2, NULL, global_work_size, 
                                 local_work_size, 0, NULL, NULL);
    
    cl_float4 *output = (cl_float4 *)malloc(h * w * sizeof(cl_float4));

    const size_t origin[3] = {0, 0, 0};
    const size_t region[3] = {w, h, 1};
    clEnqueueReadImage(queue, output_buf, CL_TRUE, origin, region, 0, 0, output, 0, NULL, NULL);
        
    printf("top left: %.2f\n", output[0].x);

    free(output);
    clReleaseMemObject(glyph_data_buf);
    clReleaseMemObject(output_buf);
    clReleaseKernel(msdf_kernel);
    clReleaseProgram(program);
    clReleaseCommandQueue(queue);
    clReleaseContext(ctx);
    return 0;
}
