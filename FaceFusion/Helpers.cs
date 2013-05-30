using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;

namespace FaceFusion
{
    static class Helpers
    {

        public static unsafe short[] ConvertDepthImagePixelToShort(DepthImagePixel[] depthImage)
        {
            int len = depthImage.Length;
            short[] ret = new short[len];

            fixed (short* retPtrFixed = ret)
            {
                fixed (DepthImagePixel* srcPtrFixed = depthImage)
                {
                    short* retPtr = retPtrFixed;
                    DepthImagePixel* srcPtr = srcPtrFixed;

                    for (int i = 0; i < len; i++)
                    {
                        *(retPtr) = (*(srcPtr)).Depth;
                        retPtr++;
                        srcPtr++;

                        //ret[i] = depthImage[i].Depth;
                    }
                }
            }

            return ret;
        }
    }
}
