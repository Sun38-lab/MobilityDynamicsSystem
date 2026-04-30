using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MobilityMonitor
{
    public class LogData
    {
        public double Time { get; set; }
        public double TargetPitch { get; set; }
        public double ActualPitch { get; set; }
        public double Pwm { get; set; }
    }
}
