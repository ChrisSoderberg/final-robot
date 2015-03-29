package rp.robotics.mapping;

import java.awt.geom.Point2D;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.geom.Rectangle;
import lejos.robotics.navigation.Pose;
import rp.robotics.mapping.IGridMap;
import rp.robotics.mapping.RPLineMap;


public class MazeGridMap implements rp.robotics.mapping.IGridMap{
	
	private RPLineMap lineMap;
	private int gridXSize;
	private int gridYSize;
	private float offsetX;
	private float offsetY;
	private float cellSize;
	private Point2D[][] pointsGrid;

	public MazeGridMap(RPLineMap lineMap, int xsize, int ysize, float cellsize, float offsetX, float offsetY)
	{
		this.lineMap = lineMap;
		this.gridXSize = xsize;
		this.gridYSize = ysize;
		this.cellSize = cellsize;
		this.offsetX = offsetX;
		this.offsetY = offsetY;
		
		
		pointsGrid = new Point[xsize][ysize];
		for(int i = 0; i < xsize; i++)
		{
			for(int j = 0; j < ysize; j++)
			{
				pointsGrid[i][j] = new Point(i * cellsize + offsetX ,j * cellsize + offsetY);
			}
		}
	}

	/**
	 * @return gridXsize return the horizontal size of the grid.
	 */
	@Override
	public int getXSize() 
	{
		return gridXSize;
	}
	/**
	 * @return gridYsize return the vertical size of the grid.
	 */
	@Override
	public int getYSize() 
	{
		return gridYSize;
	}

	@Override
	/**
	 * checks if the grid position is within the grid map and whether or not it is inside the lineMap.
	 * @param _x the x coordinate of the grid position
	 * @param _y the y coordinate of the grid position
	 * @return if the grid point exists within the grid map.
	 */
	public boolean isValidGridPosition(int _x, int _y) 
	{
		if(_x < gridXSize && _y < gridYSize)
		{
			if(_x >= 0 && _y >=0 )
			{
				if(lineMap.getBoundingRect().contains(pointsGrid[_x][_y]));
				{
					return true;
				}
			}
		}
		return false;
	}

	/**
	 * Checks if the given grid position is obstructed either by being outside of the grid map or
	 * being obstructed by the line map.
	 * @param _x the x coordinate of the grid position
	 * @param _y the y coordinate of the grid position
	 * @return whether or not the given grid position can be moved to.
	 */
	@Override
	public boolean isObstructed(int _x, int _y) 
	{
		if(isValidGridPosition(_x, _y))
		{
			if(lineMap.inside((Point) pointsGrid[_x][_y]))
			{
				return false;
			}
		}	
		return true;
	}
	/**
	 * @param _x the x coordinate of grid position
	 * @param _y the y coordinate of the grid position
	 * @return the point coordinate within the grid that this grid coordinate is at. 
	 */
	@Override
	public Point getCoordinatesOfGridPosition(int _x, int _y) 
	{
		float px = (_x * cellSize) + offsetX;
		float py = (_y * cellSize) + offsetY;
		
		Point pos = new Point(px, py);
		return pos;
	}
	/**
	 * Checks whether the given grid coordinates are valid first
	 * Checks if either grid position is obstructed
	 * returns true if coordinates are the same
	 * Checks each possible movement between grid positions either +- x or +- y
	 * Changes heading based on grid position moved to.
	 * Checks there are no obstructions in its path moving across the cell.
	 * @param _x1 the starting x coordinate
	 * @param _y1 the starting y coordinate
	 * @param _x2 the x coordinate goal 
	 * @param _y2 the y coordinate goal
	 * @return whether or not a movement by the robot between these coordinates is possible.
	 */
	@Override
	public boolean isValidTransition(int _x1, int _y1, int _x2, int _y2) 
	{
		float heading = 0; 
		if((isValidGridPosition(_x1, _y1)) && (isValidGridPosition(_x2, _y2)))
		{
			if((isObstructed(_x1,_y1)) || (isObstructed(_x2, _y2)))
			{
				return false;
			}
			if (_x1 == _x2 && _y1 == _y2)
			{
				return true;
			}
			if((_x1 == _x2 && _y1 == _y2+1)  || (_x1 == _x2 && _y1 == _y2-1) || (_x1 == _x2+1 && _y1 == _y2) || (_x1 == _x2-1 && _y1 == _y2))
			{
				if(_y1 == _y2 +1)
				{
					heading = -90;
				}	
				if(_y1 == _y2 -1)
				{
					heading = 90;
				}
				if(_x1 == _x2 +1)
				{
					heading = 180;
				}	
				if(_x1 == _x2 -1)
				{
					heading = 0;
				}
				if((rangeToObstacleFromGridPosition(_x1, _y1, heading) > cellSize))
				{
					return true;
				}
				else return false;
			}
			else return false;
		}
		else return false;
	}

	
	/**
	 * Creates a point coordinate from the robots grid position and uses this to create a pose
	 * then checks the range from that pose to a line within the line map.
	 * @param x the horizontal coordinate of the robot
	 * @param y the vertical coordinate of the robot
	 * @param _heading the direction the robot is facing
	 * @return the distance from the robot to the nearest obstruction from the line map
	 */
	@Override
	public float rangeToObstacleFromGridPosition(int x, int y, float heading) 
	{
		Point p = this.getCoordinatesOfGridPosition(x, y);
		Pose pos = new Pose(p.x, p.y, heading);
		return lineMap.range(pos);	
	}
}