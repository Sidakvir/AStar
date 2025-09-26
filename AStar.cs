using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AStar : MonoBehaviour
{
    [SerializeField] private int dimension;
    private Node[,] grid;
    [SerializeField] private Vector2Int startPos;
    [SerializeField] private Vector2Int endPos;
    public GameObject floor;
    public GameObject wall;
    public GameObject marker;
    private List<Node> openList;
    private HashSet<Node> closedList;
    [SerializeField] private bool Recompute;
    [SerializeField] private float floorThreshold;
    [SerializeField] private int seed;
    [SerializeField] private bool FromPicture;
    public void Start()
    {
        if (FromPicture)
        {
            PicturePathFind();
        } else
        {
            PathFind();
        }
    }

    public void Update()
    {
        if (Recompute && !FromPicture)
        {
            PathFind();
            Recompute = false;
        }
    }

    public void PathFind()
    {
        grid = new Node[dimension, dimension]; //A grid like the x and y axis

        closedList = new HashSet<Node>(); //Basically a list with not input index. It just compares every element in it really fast

        for (int x = 0; x < dimension; x++)
        {
            for (int y = 0; y < dimension; y++) //Loop through each x and y value in the grid
            {
                float sample = Mathf.PerlinNoise((x + seed) * .1f, (y + seed) * .1f); //Just a perlin noise function. (Not Important)
                if (sample >= floorThreshold) //Make the node Open
                {
                    grid[x, y] = new Node(new Vector2Int(x, y), Node.Cell.Open, DirectionDistance(new Vector2Int(x, y), endPos));//Set the values of a node
                    //The DirectionDistance is the h cost
                    Instantiate(floor, new Vector2(x, y), Quaternion.identity); //Instantiate or create a tile at this position
                }
                else //Make the node Closed
                {
                    grid[x, y] = new Node(new Vector2Int(x, y), Node.Cell.Closed, DirectionDistance(new Vector2Int(x, y), endPos)); //Set the values of a node
                    Instantiate(wall, new Vector2(x, y), Quaternion.identity); //Instantiate a wall
                    closedList.Add(grid[x, y]); //Add to the closed list
                }
            }
        }

        Node startNode = grid[startPos.x, startPos.y]; //Set the startNode
        startNode.g = 0; //Make the g cost (cost to go from the parent to this) 0. This happens because you spawn here and don't move.
        startNode.GetFCost(); //Set the f cost

        openList = new List<Node>() { startNode }; //Make a list, basically an array without a set max number of elements. It has an integer value that points to a Node.
        //It has startNode only, orignally.

        Node current = new Node(); //The current node. It is not set yet
        while (openList.Count > 0) //While the openList has elements
        {
            current = GetLowestFCost(openList); //Get the node with the lowest f cost, from the openList.

            if (current.position == endPos) //Tf the current position is the same as the end position
            {
                ReconstructPath(); //Explained later
                Debug.Log("Found"); //Show that it is found
                return; //End the loop and pathfinding code
            }

            openList.Remove(current); //Remove the current node from the openList, becuase its been evaluated
            closedList.Add(current); //Add the current node to the closedList, because its been evaluated

            foreach (Vector2Int pos in ReturnNeighbours(current.position)) //Get the positions of the neighbours of current (close nodes). The function is explained below.
            {
                if (!Bound(pos)) continue; //If the position is outside of the array end this foreach loop iteration
                Node neighbour = grid[pos.x, pos.y];  //Get the node value from the position value

                if (closedList.Contains(neighbour)) continue; //If this node has already been evaluated, and placed in the closed list, end this foreach loop iteration

                float GD = current.g + DirectionDistance(current.position, neighbour.position); //Basically the current path's g cost.

                if (!openList.Contains(neighbour) || GD < neighbour.g) //If the openList does not contain this neighbour, or if this new path has a lower g (distance) cost. 
                {
                    neighbour.g = GD; //Set the neighbour's g cost to GD
                    neighbour.h = DirectionDistance(neighbour.position, endPos); //Set the h cost.
                    neighbour.GetFCost(); //Calculate the f cost.
                    neighbour.parent = current; //Set the parent of the neighbour to the current node, for path reconstruction.

                    if (!openList.Contains(neighbour)) //If the openList does not contain the neighbour node 
                    {
                        openList.Add(neighbour); //Add the neighbour node
                    }
                }
            }
        }
        Debug.Log("Not Found"); //If the loop terminates without finding a path say it.
    }

    public void PicturePathFind()
    {
        ImageParser parser = GetComponent<ImageParser>();
        grid = parser.GetGridFromImage();
        Vector2Int endPicturePos = parser.GetEndPos();
        closedList = new HashSet<Node>();

        for (int x = 0; x < grid.GetLength(0); x++)
        {
            for (int y = 0; y < grid.GetLength(1); y++)
            {
                grid[x, y].h = DirectionDistance(new Vector2Int(x, y), endPicturePos);
                if (grid[x,y].cell == Node.Cell.Closed)
                {
                    closedList.Add(grid[x, y]);
                }
            }
        }
        Vector2Int startNodePos = parser.GetStartPos();
        Node startNode = grid[startNodePos.x, startNodePos.y];
        startNode.g = 0;
        startNode.GetFCost();

        openList = new List<Node>() { startNode };

        Node current = new Node();
        while (openList.Count > 0)
        {
            current = GetLowestFCost(openList);

            if (current.position == endPicturePos)
            {
                ReconstructPath(current, endPicturePos);
                Debug.Log("Found");
                return;
            }

            openList.Remove(current);
            closedList.Add(current);

            foreach (Vector2Int pos in ReturnNeighbours(current.position))
            {
                if (!Bound(pos)) continue;
                Node neighbour = grid[pos.x, pos.y];

                if (closedList.Contains(neighbour)) continue;

                float GD = current.g + DirectionDistance(current.position, neighbour.position);

                if (!openList.Contains(neighbour) || GD < neighbour.g)
                {
                    neighbour.g = GD;
                    neighbour.h = DirectionDistance(neighbour.position, endPicturePos);
                    neighbour.GetFCost();
                    neighbour.parent = current;

                    if (!openList.Contains(neighbour))
                    {
                        openList.Add(neighbour);
                    }
                }
            }
        }
        if (current != null && current.position != endPos) Debug.Log("Not Found");
    }
    public void ReconstructPath() //To get the path from the start to the end
    {
        Node current = grid[endPos.x, endPos.y]; //The current position. Begins as the end position.
        while(current != null) //If the current position exists. Important when you reach the start node.
        {
            Instantiate(marker, new Vector3(current.position.x, current.position.y, 0), Quaternion.identity); //Create a marker of the path
            current = current.parent; //Set the temporary current node to its parent
        }
    }
    public void ReconstructPath(Node curent, Vector2Int endPicturePos) 
    {
        Node current = grid[endPicturePos.x, endPicturePos.y];
        while (current != null)
        {
            Instantiate(marker, new Vector3(current.position.x, current.position.y, 0), Quaternion.identity);
            current = current.parent;
        }
    }

    public bool Bound(Vector2Int pos) //Check if the position exists
    {
        //Basically if x or y is more than zero (positive) or less than the dimension return true
        if (pos.x >= 0 && pos.y >= 0 && pos.x < dimension && pos.y < dimension) return true;
        return false; //If x or y are less than zero (negative), or more than or equal to the dimension of the grid return false
    }
    public Node GetLowestFCost(List<Node> nodes) //Get the node with the lowest f cost.
    {
        Node best = nodes[0]; //Begin with the current best node. Lets begin with the first
        foreach (Node node in nodes) //Loop through all nodes
        {
            if (best.f >= node.f) best = node; //Since we are trying to minimize the f cost, set the best node to the one with the lowest f cost
        }
        return best; //return the best node
    }
    public static float DirectionDistance(Vector2Int start, Vector2Int end) //Get the distance from a to b. This is the typical method with the Pythagorean Theorem.
    {
        float x = (start.x - end.x) * (start.x - end.x); //Square the x distance from a to b
        float y = (start.y - end.y) * (start.y - end.y); //Square the y distance from a to b

        return Mathf.Sqrt(x + y); //Square root it to find the true, straight line, or angled distance
    }

    public static Vector2Int[] ReturnNeighbours(Vector2Int pos) //Get the current position
    {
        Vector2Int[] Difference =
        {
            new Vector2Int(1,0),
            new Vector2Int(1,1),
            new Vector2Int(0,1),
            new Vector2Int(-1,1),
            new Vector2Int(-1,0),
            new Vector2Int(-1,-1),
            new Vector2Int(0,-1),
            new Vector2Int(1,-1),
        }; //Basically all the vectors/displacements to go to the neighbour

        Vector2Int[] temp = new Vector2Int[8]; //An array with 8 elements, that represent the displaced positions
        for(int i = 0; i < 8; i++) //For every element in temp
        {
            temp[i] = Difference[i] + pos; //Get the displacement
            if (temp[i].x < 0 || temp[i].y < 0) //If it is not bounded
            {
                temp[i] = Vector2Int.zero; //Return zero
            }
        }

        return temp; //Return the temp array
    }
}

public class Node
{
    public Node parent; //This is the Node that the current Node has come from
    public Vector2Int position; //The position of the Node (integer) with x and y components
    public float g; //The cost to move. Either 1 or the square root of 2
    public float h; //The distance from this node to the end
    public float f; //The true cost. g + h
    public Cell cell; //What type of grid position is this
    public enum Cell
    {
        Open = 0,//Available for movement (kinda confusing with OpenList, I know)
        Closed = 1,//Unavailable for movement
        Start = 2,//The start node
        End = 3,// The end node
    }
    
    public void GetFCost()
    {
        f = h + g;//calculate the f cost as h + g
    }

    public Node(Node Parent, Vector2Int Position, float G, float H)//Initialization method 
    {
        this.parent = Parent;
        this.position = Position;
        this.g = G;
        this.h = H;
    }
    public Node(Vector2Int Position, Cell Cell, float H)//Initialization method 
    {
        this.position = Position;
        this.cell = Cell;
        this.h = H;
    }
    public Node(Vector2Int Position, Cell Cell)//Initialization method 
    {
        this.position = Position;
        this.cell = Cell;
    }
    public Node()//Initialization method. Empty.
    {

    }
}