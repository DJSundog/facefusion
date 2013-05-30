using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;

namespace FaceFusion
{
    static class Helpers
    {

        public static short[] ConvertDepthImagePixelToShort(DepthImagePixel[] depthImage)
        {
            int len = depthImage.Length;
            short[] ret = new short[len];

            for (int i = 0; i < len; i++)
            {
                ret[i] = depthImage[i].Depth;
            }

            return ret;
        }
    }
}
