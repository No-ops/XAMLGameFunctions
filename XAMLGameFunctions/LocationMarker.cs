using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Foundation;
using Windows.UI.Xaml.Shapes;
using Windows.Foundation.Collections;
using Windows.UI;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;

namespace XAMLGameFunctions
{
    public static class LocationMarker
    {
        /// <summary>
        /// Generates a Path object covering the given tiles. Not going over the specified max tiles or below 0.
        /// </summary>
        /// <param name="tiles">Tiles to mark.</param>
        /// <param name="tileWidth">Tile side length.</param>
        /// <param name="overflow">Thickness of the edge around the marked area.</param>
        /// <param name="areaXCount">Width of area to draw on.</param>
        /// <param name="areaYCount">Height of area to draw on.</param>
        /// <returns>Path object covering the given tiles.</returns>
        public static Path GenerateCoverage(PointInt[] tiles, double tileWidth, double overflow, int areaXCount,
            int areaYCount)
        {
            if (overflow < 0)
                return InternalGenerateCoverage(tiles, tileWidth, overflow, 0, 0, true);

            foreach (var tile in tiles)
            {
                if (tile.X < 0 || tile.Y < 0 || tile.X >= areaXCount || tile.Y >= areaYCount)
                    throw new ArgumentException(
                        string.Format("The tile {0} was outside the bounds of the specified area.", tile));
            }

            return InternalGenerateCoverage(tiles, tileWidth, overflow, areaXCount, areaYCount, false);
        }

        /// <summary>
        /// Generates a Path object covering the given tiles.
        /// </summary>
        /// <param name="tiles">Tiles to mark.</param>
        /// <param name="tileWidth">Tile side length.</param>
        /// <param name="overflow">Thickness of the edge around the marked area.</param>
        /// <returns>Path object covering the given tiles.</returns>
        public static Path GenerateCoverage(PointInt[] tiles, double tileWidth, double overflow)
        {
            return InternalGenerateCoverage(tiles, tileWidth, overflow, 0, 0, true);
        }

        /// <summary>
        /// Generates a Path object covering the given tiles with wobbly edges. Not going over the specified max tiles or below 0.
        /// </summary>
        /// <param name="tiles">Tiles to mark.</param>
        /// <param name="tileWidth">Tile side length.</param>
        /// <param name="overflow">Thickness of the edge around the marked area.</param>
        /// <param name="areaXCount">Width of area to draw on.</param>
        /// <param name="areaYCount">Height of area to draw on.</param>
        /// <param name="wobbliness">Wobbliness of edges.</param>
        /// <returns>Path object covering the given tiles.</returns>
        public static Path GenerateCoverage(PointInt[] tiles, double tileWidth, double overflow, int areaXCount,
            int areaYCount, double wobbliness)
        {
            if (overflow < 0)
                return InternalGenerateCoverage(tiles, tileWidth, overflow, 0, 0, true);

            foreach (var tile in tiles)
            {
                if (tile.X < 0 || tile.Y < 0 || tile.X >= areaXCount || tile.Y >= areaYCount)
                    throw new ArgumentException(
                        string.Format("The tile {0} was outside the bounds of the specified area.", tile));
            }

            return InternalGenerateCoverage(tiles, tileWidth, overflow, areaXCount, areaYCount, false, wobbliness);
        }

        /// <summary>
        /// Generates a Path object covering the given tiles with wobbly edges.
        /// </summary>
        /// <param name="tiles">Tiles to mark.</param>
        /// <param name="tileWidth">Tile side length.</param>
        /// <param name="overflow">Thickness of the edge around the marked area.</param>
        /// <param name="wobbliness">Wobbliness of edges.</param>
        /// <returns>Path object covering the given tiles.</returns>
        public static Path GenerateCoverage(PointInt[] tiles, double tileWidth, double overflow, double wobbliness)
        {
            return InternalGenerateCoverage(tiles, tileWidth, overflow, 0, 0, true, wobbliness);
        }




        static Path InternalGenerateCoverage(PointInt[] tiles, double tileWidth, double overflow, int areaXCount, int areaYCount, bool overflowEdges = true, double wobbliness = 0)
        {
            bool negativeOverflow = overflow < 0;

            List<PointInt> tilesToSort = tiles.ToList();
            List<PointInt[]> discreteAreas = new List<PointInt[]>();

            while (tilesToSort.Count > 0)
            {
                List<PointInt> currentArea = new List<PointInt>();
                PointInt currentTile = tilesToSort[0];
                currentArea.Add(tilesToSort[0]);
                tilesToSort.RemoveAt(0);
                GetAllNeighbours(currentTile, currentArea, tilesToSort, negativeOverflow);
                discreteAreas.Add(currentArea.ToArray<PointInt>());
            }

            List<PointInt[]> holes = new List<PointInt[]>();

            for (int i = 0; i < discreteAreas.Count; i++)
            {
                List<PointInt> areaList = discreteAreas[i].ToList();
                PointInt[][] holesToAdd = GetAllHoles(areaList, negativeOverflow);
                if (holesToAdd.Length > 0)
                    holes.AddRange(holesToAdd);
                discreteAreas[i] = areaList.ToArray();
            }

            PathFigureCollection pathFigureCollection = new PathFigureCollection();
            foreach (PointInt[] area in discreteAreas)
            {
                pathFigureCollection.Add(GeneratePathFigure(overflow, area, tileWidth, areaXCount, areaYCount, overflowEdges, false, wobbliness));
            }

            foreach (PointInt[] hole in holes)
            {
                pathFigureCollection.Add(GeneratePathFigure(overflow * -1, hole, tileWidth, areaXCount, areaYCount, true, true, wobbliness));
            }

            PathGeometry pathGeometry = new PathGeometry();
            pathGeometry.Figures = pathFigureCollection;
            Path path = new Path();
            path.Data = pathGeometry;

            return path;
        }

        static PointInt[][] GetAllHoles(List<PointInt> area, bool checkDiagonals)
        {
            int minX = area[0].X;
            int maxX = area[0].X;
            int minY = area[0].Y;
            int maxY = area[0].Y;

            foreach (PointInt tile in area)
            {
                minX = Math.Min(tile.X, minX);
                maxX = Math.Max(tile.X, maxX);
                minY = Math.Min(tile.Y, minY);
                maxY = Math.Max(tile.Y, maxY);
            }

            List<PointInt[]> holes = new List<PointInt[]>();

            for (int x = minX; x <= maxX; x++)
            {
                for (int y = minY; y <= maxY; y++)
                {
                    PointInt currentTile = new PointInt(x, y);
                    List<PointInt> hole = new List<PointInt>();
                    if (!IsTileInArray(currentTile, area.ToArray()) && !TileCanReachEdge(currentTile, area, minX, minY, maxX, maxY, hole, checkDiagonals))
                    {
                        holes.Add(hole.ToArray());
                        area.AddRange(hole);
                    }
                }
            }

            return holes.ToArray();
        }

        static bool TileCanReachEdge(PointInt tile, List<PointInt> filledArea, int minX, int minY, int maxX, int maxY, List<PointInt> checkedTiles, bool checkDiagonals)
        {
            bool tileIsOnEdge = (tile.X == minX || tile.X == maxX || tile.Y == minY || tile.Y == maxY);
            if (tileIsOnEdge)
                return true;

            bool foundEdge = false;

            checkedTiles.Add(tile);
            PointInt straightVector = new PointInt(1, 0);
            PointInt diagonalVector = new PointInt(1, 1);

            for (int i = 0; i < 4 && !foundEdge; i++)
            {
                PointInt tileToCheck = new PointInt(tile.X + straightVector.X, tile.Y + straightVector.Y);
                if (!checkedTiles.Contains(tileToCheck) && !IsTileInArray(tileToCheck, filledArea.ToArray()))
                {
                    foundEdge = TileCanReachEdge(tileToCheck, filledArea, minX, minY, maxX, maxY, checkedTiles, checkDiagonals);
                }

                if (checkDiagonals && !foundEdge)
                {
                    tileToCheck = new PointInt(tile.X + diagonalVector.X, tile.Y + diagonalVector.Y);
                    if (!checkedTiles.Contains(tileToCheck) && !IsTileInArray(tileToCheck, filledArea.ToArray()))
                    {
                        foundEdge = TileCanReachEdge(tileToCheck, filledArea, minX, minY, maxX, maxY, checkedTiles, checkDiagonals);
                    }
                }

                straightVector = RotateVector90(straightVector, true);
                diagonalVector = RotateVector90(diagonalVector, true);
            }

            return foundEdge;
        }

        // Starts with startTile, assumes startTile is already in currentArea
        static void GetAllNeighbours(PointInt startTile, List<PointInt> currentArea, List<PointInt> listToSort, bool skipDiagonals)
        {
            PointInt straightVector = new PointInt(1, 0);
            PointInt diagonalVector = new PointInt(1, 1);

            for (int i = 0; i < 4; i++)
            {
                straightVector = RotateVector90(straightVector, true);
                diagonalVector = RotateVector90(diagonalVector, true);

                PointInt coordinateToTest = new PointInt(startTile.X + straightVector.X, startTile.Y + straightVector.Y);

                if (listToSort.Contains(coordinateToTest))
                {
                    currentArea.Add(coordinateToTest);
                    listToSort.Remove(coordinateToTest);
                    GetAllNeighbours(coordinateToTest, currentArea, listToSort, skipDiagonals);
                }

                if (!skipDiagonals)
                {
                    coordinateToTest = new PointInt(startTile.X + diagonalVector.X, startTile.Y + diagonalVector.Y);

                    if (listToSort.Contains(coordinateToTest))
                    {
                        currentArea.Add(coordinateToTest);
                        listToSort.Remove(coordinateToTest);
                        GetAllNeighbours(coordinateToTest, currentArea, listToSort, skipDiagonals);
                    }
                }
            }
        }

        static PathFigure GeneratePathFigure(double overflow, PointInt[] tiles, double tileWidth, int x, int y, bool overflowEdges, bool drawingHole, double wobbliness)
        {
            // Find first tile side not blocked by another tile
            bool freeTileFound = false;
            int tileIndex = 0;
            PointInt side = new PointInt();
            PointInt drawDirection = new PointInt();

            while (!freeTileFound)
            {
                for (int i = 0; i < 4 && !freeTileFound; i++)
                {
                    // This code sets the coordinates (0,-1),(-1,0),(0,1),(1,0) when i goes from 0 to 3
                    side.X = (i % 2) * (i - 2);
                    side.Y = ((i + 1) % 2) * (i - 1);

                    if (!IsTileInArray(new PointInt(tiles[tileIndex].X + side.X, tiles[tileIndex].Y + side.Y), tiles))
                    {
                        drawDirection = RotateVector90(side, false);
                        freeTileFound = true;
                    }
                }
                if (!freeTileFound)
                    tileIndex++;
            }



            PointInt currentTile = new PointInt(tiles[tileIndex].X, tiles[tileIndex].Y);

            bool isDrawingOnEdge = (currentTile.X + side.X >= x || currentTile.X + side.X < 0 || currentTile.Y + side.Y >= y || currentTile.Y + side.Y < 0);
            Point startPoint = GetStandardFirstPointForSide(currentTile, side, drawDirection, tileWidth, overflow, isDrawingOnEdge && !overflowEdges);

            // If overflow is positive and we're coming from a concave turn adjust start point
            if (overflow > 0 && IsTileInArray(new PointInt(tiles[tileIndex].X + side.X - drawDirection.X, tiles[tileIndex].Y + side.Y - drawDirection.Y), tiles))
            {
                startPoint.X += 2 * overflow * drawDirection.X;
                startPoint.Y += 2 * overflow * drawDirection.Y;
            }
            // if overflow is negative and we're coming from a convex turn adjust start point
            else if (overflow < 0 && !IsTileInArray(new PointInt(currentTile.X - drawDirection.X, currentTile.Y - drawDirection.Y), tiles))
            {
                startPoint.X -= 2 * overflow * drawDirection.X;
                startPoint.Y -= 2 * overflow * drawDirection.Y;
            }

            // If direction is coming from an edge and we don't overflow edges adjust start point accordingly.
            if (!overflowEdges && (currentTile.X - drawDirection.X >= x || currentTile.X - drawDirection.X < 0 || currentTile.Y - drawDirection.Y >= y || currentTile.Y - drawDirection.Y < 0))
            {
                startPoint.X += drawDirection.X * overflow;
                startPoint.Y += drawDirection.Y * overflow;
            }

            Point lastDrawnPoint = new Point(startPoint.X, startPoint.Y);


            PathSegmentCollection pathSegmentCollection = new PathSegmentCollection();

            do
            {
                lastDrawnPoint = DrawNextSegments(pathSegmentCollection, ref currentTile, tiles, ref side, ref drawDirection, overflow, tileWidth, x, y, lastDrawnPoint, wobbliness, overflowEdges, drawingHole);
            } while (lastDrawnPoint != startPoint);

            PathFigure pathFigure = new PathFigure();
            pathFigure.StartPoint = startPoint;
            pathFigure.Segments = pathSegmentCollection;

            return pathFigure;
        }

        static Point GetStandardFirstPointForSide(PointInt tile, PointInt side, PointInt drawDirection, double tileWidth, double overflow, bool isDrawingOnEdge = false)
        {
            PointInt cornerPointInt = new PointInt();

            cornerPointInt.X = side.X;
            cornerPointInt.Y = side.Y;

            if (cornerPointInt.X == 0)
            {
                cornerPointInt.X = drawDirection.X * -1;
            }
            else if (cornerPointInt.Y == 0)
            {
                cornerPointInt.Y = drawDirection.Y * -1;
            }

            Point retPoint = GetCorner(tile, tileWidth, cornerPointInt);

            if (!isDrawingOnEdge)
            {
                retPoint.X += side.X * overflow;
                retPoint.Y += side.Y * overflow;
            }

            return retPoint;
        }

        static Point GetCorner(PointInt tile, double tileWidth, PointInt cornerPointInt)
        {
            double retx;
            double rety;

            retx = tileWidth * tile.X;
            rety = tileWidth * tile.Y;

            retx += Math.Max(cornerPointInt.X, 0) * tileWidth;
            rety += Math.Max(cornerPointInt.Y, 0) * tileWidth;

            return new Point(retx, rety);
        }


        static PointInt RotateVector90(PointInt pointInt, bool clockwise)
        {
            PointInt retVector = new PointInt();
            if (clockwise)
            {
                retVector.X = -1 * pointInt.Y;
                retVector.Y = pointInt.X;
            }
            else
            {
                retVector.X = pointInt.Y;
                retVector.Y = -1 * pointInt.X;
            }
            return retVector;
        }

        static bool IsTileInArray(PointInt tile, PointInt[] list)
        {
            bool tileFound = false;
            for (int i = 0; i < list.Length && !tileFound; i++)
            {
                if (tile == list[i])
                    tileFound = true;
            }

            return tileFound;
        }

        static Point DrawNextSegments(PathSegmentCollection pathSegmentCollection, ref PointInt tile, PointInt[] tileList, ref PointInt side, ref PointInt direction, double overflow, double tileWidth, int boardX, int boardY, Point lastPoint, double wobbliness, bool overflowEdges, bool drawingHole)
        {
            // Returns end point of the segments added
            // Changes direction, side and tile to the next segment to be drawn

            bool isDrawingOnEdge = (tile.X + side.X >= boardX || tile.X + side.X < 0 || tile.Y + side.Y >= boardY || tile.Y + side.Y < 0);
            Point segment2End;


            // Check if there's a tile diagonaly from current tile or straight in the drawing direction
            // if not draw corner on current tile
            // Set direction, tile and side for next place to draw on
            bool tileFoundDiagonaly =
                IsTileInArray(new PointInt(tile.X + direction.X + side.X, tile.Y + direction.Y + side.Y), tileList);
            bool tileFoundAhead = IsTileInArray(new PointInt(tile.X + direction.X, tile.Y + direction.Y), tileList);
            bool overflowIsPositive = overflow >= 0;
            if (tileFoundDiagonaly && (!drawingHole || tileFoundAhead) && (overflowIsPositive || tileFoundAhead))
            { // Tile found diagonaly from current tile
                Point lineEnd = GetStandardLineEnd(tile, side, direction, overflow, tileWidth);
                // Shorten first line on concave corner if overflow is positive
                if (overflow > 0)
                {
                    lineEnd.X -= direction.X * 2 * overflow;
                    lineEnd.Y -= direction.Y * 2 * overflow;
                }

                if (wobbliness > 0)
                    DrawCurvedLine(pathSegmentCollection, lineEnd, lastPoint, wobbliness, direction);
                else
                    DrawLine(pathSegmentCollection, lineEnd);

                tile.X += direction.X + side.X;
                tile.Y += direction.Y + side.Y;

                PointInt newSide = RotateVector90(side, true);
                side.X = newSide.X;
                side.Y = newSide.Y;

                PointInt newDirection = RotateVector90(direction, true);
                direction.X = newDirection.X;
                direction.Y = newDirection.Y;

                segment2End = GetStandardFirstPointForSide(tile, side, direction, tileWidth, overflow);

                // On positive overflow fix arc endpoint
                if (overflow > 0)
                {
                    segment2End.X += direction.X * overflow * 2;
                    segment2End.Y += direction.Y * overflow * 2;
                }

                ArcSegment arc = new ArcSegment();
                arc.SweepDirection = SweepDirection.Clockwise;
                arc.Size = new Size(Math.Abs(overflow), Math.Abs(overflow));
                arc.Point = segment2End;
                pathSegmentCollection.Add(arc);
            }
            else if (IsTileInArray(new PointInt(tile.X + direction.X, tile.Y + direction.Y), tileList))
            { // Tile found straight ahead from current tile
                Point lineEnd = GetStandardLineEnd(tile, side, direction, overflow, tileWidth, isDrawingOnEdge && !overflowEdges);

                if (wobbliness>0)
                    DrawCurvedLine(pathSegmentCollection, lineEnd, lastPoint, wobbliness, direction);
                else
                    DrawLine(pathSegmentCollection, lineEnd);

                tile.X += direction.X;
                tile.Y += direction.Y;

                segment2End = lineEnd;

                // This code is only needed if cornerRadius parameter is added, right now the line length would be 0
                //LineSegment line2 = new LineSegment();
                //line2.Point = segment2End;
                //pathSegmentCollection.Add(line2);
            }
            else
            { // No other tiles to continue on to found, drawing around the corner on current tile
                Point lineEnd = GetStandardLineEnd(tile, side, direction, overflow, tileWidth, isDrawingOnEdge && !overflowEdges);
                bool cornerLeadsToEdge = (tile.X + direction.X >= boardX || tile.X + direction.X < 0 || tile.Y + direction.Y >= boardY || tile.Y + direction.Y < 0);

                // Shorten first line on convex corner if overflow is negative
                if (overflow < 0)
                {
                    lineEnd.X += direction.X * 2 * overflow;
                    lineEnd.Y += direction.Y * 2 * overflow;
                }

                if (cornerLeadsToEdge && !overflowEdges)
                { // If this corner leads to the end of the board and we don't overflow edges shorten line
                    lineEnd.X -= direction.X * overflow;
                    lineEnd.Y -= direction.Y * overflow;
                }

                if (wobbliness>0)
                    DrawCurvedLine(pathSegmentCollection, lineEnd, lastPoint, wobbliness, direction);
                else
                    DrawLine(pathSegmentCollection, lineEnd);

                PointInt newSide = RotateVector90(side, false);
                side.X = newSide.X;
                side.Y = newSide.Y;

                PointInt newDirection = RotateVector90(direction, false);
                direction.X = newDirection.X;
                direction.Y = newDirection.Y;

                segment2End = GetStandardFirstPointForSide(tile, side, direction, tileWidth, overflow);

                // On negative overflow fix arc endpoint
                if (overflow < 0)
                {
                    segment2End.X -= direction.X * overflow * 2;
                    segment2End.Y -= direction.Y * overflow * 2;
                }

                if (cornerLeadsToEdge && !overflowEdges)
                {
                    segment2End.X -= side.X * overflow;
                    segment2End.Y -= side.Y * overflow;
                }
                else if (isDrawingOnEdge && !overflowEdges)
                {
                    segment2End.X += direction.X * overflow;
                    segment2End.Y += direction.Y * overflow;
                }


                ArcSegment arc = new ArcSegment();
                arc.SweepDirection = SweepDirection.Counterclockwise;
                arc.Size = new Size(Math.Abs(overflow), Math.Abs(overflow));
                arc.Point = segment2End;
                pathSegmentCollection.Add(arc);
            }


            return segment2End;
        }

        private static void DrawCurvedLine(PathSegmentCollection pathSegmentCollection, Point lineEnd, Point lineStart, double wobbliness, PointInt direction)
        {
            BezierSegment curvedLine = new BezierSegment();
            curvedLine.Point3 = lineEnd;

            double lineLength = Math.Max(Math.Abs(lineEnd.X - lineStart.X), Math.Abs(lineEnd.Y - lineStart.Y));

            if (direction.X == 0)
            {
                curvedLine.Point1 = new Point(lineEnd.X + wobbliness, lineStart.Y + (direction.Y * lineLength / 3));
                curvedLine.Point2 = new Point(lineEnd.X - wobbliness, lineEnd.Y - (direction.Y * lineLength / 3));
            }
            else
            {
                curvedLine.Point1 = new Point(lineStart.X + (direction.X * lineLength / 3), lineEnd.Y - wobbliness);
                curvedLine.Point2 = new Point(lineEnd.X - (direction.X * lineLength / 3), lineEnd.Y + wobbliness);
            }

            pathSegmentCollection.Add(curvedLine);
        }

        private static Point GetStandardLineEnd(PointInt tile, PointInt side, PointInt direction, double overflow, double tileWidth, bool isDrawingOnEdge = false)
        {
            PointInt cornerPointInt = new PointInt();
            cornerPointInt.X = side.X + direction.X;
            cornerPointInt.Y = side.Y + direction.Y;

            Point lineEnd = GetCorner(tile, tileWidth, cornerPointInt);
            if (!isDrawingOnEdge)
            {
                lineEnd.X += side.X * overflow;
                lineEnd.Y += side.Y * overflow;
            }
            return lineEnd;
        }

        static void DrawLine(PathSegmentCollection pathSegmentCollection, Point lineEnd)
        {
            LineSegment line = new LineSegment();
            line.Point = lineEnd;

            pathSegmentCollection.Add(line);
        }
    }
}
