using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Blake.NUI.WPF.Utility;
using Microsoft.Kinect;

namespace FaceFusion.ViewModels
{
    struct KinectFormat
    {
        public DepthImageFormat DepthImageFormat;
        public ColorImageFormat ColorImageFormat;
        public int NumSkeletons;
    }

    class KinectFrameWorkItem : PoolItem<KinectFormat>
    {
        public DepthImagePixel[] DepthPixels { get; private set; }
        public byte[] ColorPixels { get; private set; }
        public Skeleton[] Skeletons { get; private set; }

        public KinectFrameWorkItem(KinectFormat format, 
                                   DepthImagePixel[] depthPixels,
                                   byte[] colorPixels,
                                   Skeleton[] skeletons)
            : base(format)
        {
            if (depthPixels == null)
            {
                throw new ArgumentNullException("depthPixels");
            }
            if (colorPixels == null)
            {
                throw new ArgumentNullException("colorPixels");
            }
            if (skeletons == null)
            {
                throw new ArgumentNullException("skeletons");
            }
            this.DepthPixels = depthPixels;
            this.ColorPixels = colorPixels;
            this.Skeletons = skeletons;
        }

        public static KinectFrameWorkItem Create(KinectFormat format)
        {
            var depthSize = FormatHelper.GetDepthSize(format.DepthImageFormat);
            var colorSize = FormatHelper.GetColorSize(format.ColorImageFormat);

            var depthPixels = new DepthImagePixel[(int)(depthSize.Width * depthSize.Height)];
            
            var colorPixels = new byte[(int)(colorSize.Width * colorSize.Height * 4)];

            var skeletons = new Skeleton[format.NumSkeletons];

            return new KinectFrameWorkItem(format, depthPixels, colorPixels, skeletons);
        }
    }
}
