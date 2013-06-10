using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Blake.NUI.WPF.Utility;
using Microsoft.Kinect;

namespace FaceFusion.ViewModels
{
    public struct KinectFormat
    {
        public DepthImageFormat DepthImageFormat;
        public ColorImageFormat ColorImageFormat;
        public int NumSkeletons;
    }

    public class KinectFrameWorkItem : PoolItem<KinectFormat>
    {
        public DepthImagePixel[] DepthImagePixels { get; private set; }
        public byte[] ColorPixels { get; private set; }
        public Skeleton[] Skeletons { get; private set; }
        public int FrameNumber { get; set; }

        public DepthImagePoint[] ColorMappedToDepthPoints { get; private set; }

        public KinectFrameWorkItem(KinectFormat format, 
                                   DepthImagePixel[] depthImagePixels,
                                   byte[] colorPixels,
                                   Skeleton[] skeletons,
                                   DepthImagePoint[] colorMappedToDepthPoints)
            : base(format)
        {
            if (depthImagePixels == null)
            {
                throw new ArgumentNullException("depthImagePixels");
            }
            if (colorPixels == null)
            {
                throw new ArgumentNullException("colorPixels");
            }
            if (skeletons == null)
            {
                throw new ArgumentNullException("skeletons");
            }
            if (colorMappedToDepthPoints == null)
            {
                throw new ArgumentNullException("colorMappedToDepthPoints");
            }

            this.DepthImagePixels = depthImagePixels;
            this.ColorPixels = colorPixels;
            this.Skeletons = skeletons;
            this.ColorMappedToDepthPoints = colorMappedToDepthPoints;
        }

        public static KinectFrameWorkItem Create(KinectFormat format)
        {
            var depthSize = FormatHelper.GetDepthSize(format.DepthImageFormat);
            var colorSize = FormatHelper.GetColorSize(format.ColorImageFormat);

            var depthPixels = new DepthImagePixel[(int)(depthSize.Width * depthSize.Height)];
            
            int colorLen = (int)(colorSize.Width * colorSize.Height);
            var colorPixels = new byte[colorLen * 4];

            var skeletons = new Skeleton[format.NumSkeletons];

            var colorMappedToDepthPoints = new DepthImagePoint[colorLen];

            return new KinectFrameWorkItem(format, depthPixels, colorPixels, skeletons, colorMappedToDepthPoints);
        }
    }
}
