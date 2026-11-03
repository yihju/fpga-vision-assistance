/* Fused byte-exact cv::inRange predicates on OpenCV UINT8 HSV pixels. */
#include <stdint.h>
#include <stddef.h>

int adas_lamp_masks(const uint8_t *restrict hsv,size_t width,size_t height,
                    uint8_t *restrict planes,size_t out_size)
{
    if (!hsv || !planes || !width || !height || width>8192 || height>8192) return -1;
    size_t n=width*height;
    if (n>16777216U || out_size<n*5U) return -2;
    uint8_t *red=planes,*green=planes+n,*yellow=planes+2*n,*weak=planes+3*n,*dark=planes+4*n;
    for (size_t i=0;i<n;++i) {
        uint8_t h=hsv[3*i],s=hsv[3*i+1],v=hsv[3*i+2];
        unsigned red_hue=(h<=12U)|((h>=167U)&(h<=179U));
        unsigned strong=(s>=85U)&(v>=85U);
        red[i]=(uint8_t)(255U*(red_hue&strong));
        green[i]=(uint8_t)(255U*((h>=35U)&(h<=95U)&strong));
        yellow[i]=(uint8_t)(255U*((h>=15U)&(h<=34U)&strong));
        weak[i]=(uint8_t)(255U*(red_hue&(s>=35U)&(v>=85U)));
        dark[i]=(uint8_t)(255U*((h<=179U)&(v<=109U)));
    }
    return 0;
}
