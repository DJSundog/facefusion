using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using Microsoft.Kinect;

namespace FaceFusion.ViewModels
{
    static class FormatHelper
    {
        /// <summary>
        /// Get the depth image size from the input depth image _currentKinectFormat.
        /// </summary>
        /// <param name="imageFormat">The depth image _currentKinectFormat.</param>
        /// <returns>The width and height of the input depth image _currentKinectFormat.</returns>
        public static Size GetColorSize(ColorImageFormat imageFormat)
        {
            switch (imageFormat)
            {
                case ColorImageFormat.InfraredResolution640x480Fps30:
                case ColorImageFormat.RawBayerResolution640x480Fps30:
                case ColorImageFormat.RawYuvResolution640x480Fps15:
                case ColorImageFormat.RgbResolution640x480Fps30:
                case ColorImageFormat.YuvResolution640x480Fps15:
                    return new Size(640, 480);

                case ColorImageFormat.RawBayerResolution1280x960Fps12:
                case ColorImageFormat.RgbResolution1280x960Fps12:
                    return new Size(1280, 960);

                case ColorImageFormat.Undefined:
                    return new Size(0, 0);
            }

            throw new ArgumentOutOfRangeException("imageFormat");
        }

        /// <summary>
        /// Get the depth image size from the input depth image _currentKinectFormat.
        /// </summary>
        /// <param name="imageFormat">The depth image _currentKinectFormat.</param>
        /// <returns>The width and height of the input depth image _currentKinectFormat.</returns>
        public static Size GetDepthSize(DepthImageFormat imageFormat)
        {
            switch (imageFormat)
            {
                case DepthImageFormat.Resolution320x240Fps30:
                    return new Size(320, 240);

                case DepthImageFormat.Resolution640x480Fps30:
                    return new Size(640, 480);

                case DepthImageFormat.Resolution80x60Fps30:
                    return new Size(80, 60);
                case DepthImageFormat.Undefined:
                    return new Size(0, 0);
            }

            throw new ArgumentOutOfRangeException("imageFormat");
        }
    }
}
