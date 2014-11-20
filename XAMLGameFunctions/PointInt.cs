using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace XAMLGameFunctions
{
    public struct PointInt
    {
        public PointInt(int x, int y)
            : this()
        {
            X = x;
            Y = y;
        }

        public void Offset(int x, int y)
        {
            X += x;
            Y += y;
        }

        public void Offset(PointInt point)
        {
            Offset(point.X, point.Y);
        }

        public static bool operator ==(PointInt point1, PointInt point2)
        {
            return ((point1.X == point2.X) && (point1.Y == point2.Y));
        }

        public static bool operator !=(PointInt point1, PointInt point2)
        {
            return !(point1 == point2);
        }

        public override string ToString()
        {
            return string.Format("X:{0} Y:{1}", X, Y);
        }

        override public int GetHashCode()
        {
            return X ^ Y;
        }

        public override bool Equals(object obj)
        {
            PointInt o = (PointInt)obj;
            return X == o.X && Y == o.Y;
        }

        public int X { get; set; }
        public int Y { get; set; }
    }
}
