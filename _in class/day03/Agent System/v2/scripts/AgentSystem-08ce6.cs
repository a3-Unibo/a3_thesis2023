using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

using System.Threading.Tasks;


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public abstract class Script_Instance_08ce6 : GH_ScriptInstance
{
  #region Utility functions
  /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
  /// <param name="text">String to print.</param>
  private void Print(string text) { /* Implementation hidden. */ }
  /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
  /// <param name="format">String format.</param>
  /// <param name="args">Formatting parameters.</param>
  private void Print(string format, params object[] args) { /* Implementation hidden. */ }
  /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj) { /* Implementation hidden. */ }
  /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj, string method_name) { /* Implementation hidden. */ }
  #endregion

  #region Members
  /// <summary>Gets the current Rhino document.</summary>
  private readonly RhinoDoc RhinoDocument;
  /// <summary>Gets the Grasshopper document that owns this script.</summary>
  private readonly GH_Document GrasshopperDocument;
  /// <summary>Gets the Grasshopper script component that owns this script.</summary>
  private readonly IGH_Component Component;
  /// <summary>
  /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
  /// Any subsequent call within the same solution will increment the Iteration count.
  /// </summary>
  private readonly int Iteration;
  #endregion
  /// <summary>
  /// This procedure contains the user code. Input parameters are provided as regular arguments,
  /// Output parameters as ref arguments. You don't have to assign output parameters,
  /// they will have a default value.
  /// </summary>
  #region Runscript
  private void RunScript(bool reset, bool go, int uT, List<Point3d> P, List<Vector3d> V, double nR, double aoV, double coS, double alS, double seS, double seR, ref object oP, ref object oV, ref object oT)
  {
    // initialization
    if (reset || agSys == null)
      agSys = new AgentSystem(P, V);

    // simulation
    if (go)
    {
      // update live variables
      agSys.NeighborhoodRadius = nR;
      agSys.CohesionStrength = coS;
      agSys.AlignmentStrength = alS;
      agSys.SeparationStrength = seS;
      agSys.SeparationRadius = seR;
      agSys.AngleOfVision = ToRadians(aoV);

      // update simulation
      switch (uT)
      {
        case 0:
          agSys.Update();
          // other instructions....
          break;
        case 1:
          agSys.UpdateParallel();
          break;
        case 2:
          agSys.UpdateRTree();
          break;
        default:
          goto case 0;
      }

      Component.ExpireSolution(true);
    }


    // output
    //List<Point3d> outPos = new List<Point3d>();
    //List<Vector3d> outVel = new List<Vector3d>();
    //List<Polyline> outTrails = new List<Polyline>();

    //foreach (Agent agent in agSys.agents)
    //{
    //  outPos.Add(agent.position);
    //  outVel.Add(agent.velocity);
    //  outTrails.Add(agent.trail);
    //}

    agSys.Output();

    oP = agSys.outputValues.positions; //outPos;
    oV = agSys.outputValues.velocities; //outVel;
    oT = agSys.outputValues.trails; //outTrails;
  }
  #endregion
  #region Additional

  // global variables
  AgentSystem agSys;

  // helper classes
  public struct OutputValues
  {
    public GH_Point[] positions;
    public GH_Vector[] velocities;
    public Polyline[] trails;

    public OutputValues(int agentsCount)
    {
      positions = new GH_Point[agentsCount];
      velocities = new GH_Vector[agentsCount];
      trails = new Polyline[agentsCount];
    }
  }

  // simulation classes
  public class AgentSystem
  {
    // fields
    public List<Agent> agents;
    public double NeighborhoodRadius;
    public double CohesionStrength;

    public double BoundingBoxSize;
    public double ContainmentStrength;
    public double AlignmentStrength;
    public double SeparationStrength;
    public double SeparationRadius;
    public double AngleOfVision;
    public int iterationCount;
    public OutputValues outputValues;

    // constructor
    public AgentSystem(List<Point3d> points, List<Vector3d> vectors)
    {
      agents = new List<Agent>();

      for (int i = 0; i < points.Count; i++)
      {
        //Agent ag = new Agent(points[i], vectors[i]);
        //ag.agentSystem = this;
        Agent ag = new Agent(points[i], vectors[i], this);

        agents.Add(ag);
      }

      BoundingBoxSize = 30.0;
      ContainmentStrength = 1.0;
      iterationCount = 0;
    }

    // methods

    public void UpdateRTree()
    {
      // build RTree
      RTree pointsRTree = new RTree();
      for (int i = 0; i < agents.Count; i++)
        pointsRTree.Insert(agents[i].position, i);

      // find neighbours & call compute
      foreach (Agent agent in agents)
      {
        List<Agent> neighbours = new List<Agent>();

        // query RTree
        pointsRTree.Search(new Sphere(agent.position, NeighborhoodRadius), (sender, args) =>
        {
          Agent neighbour = agents[args.Id];
          // if it's not me
          if (neighbour != agent)
          {
            Vector3d toNeighbour = neighbour.position - agent.position;
            if (Vector3d.VectorAngle(toNeighbour, agent.velocity) < AngleOfVision)
              neighbours.Add(neighbour);
          }
        });

        // compute agent
        agent.ComputeFlocking(neighbours);
      }

      // update velocity and position
      Parallel.ForEach(agents, agent =>
      {
        agent.UpdateVelocityAndPosition();
      });

      iterationCount++;
    }

    public void Update()
    {
      // compute agent
      foreach (Agent agent in agents)
      {
        agent.ComputeFlocking(FindNeighbours(agent));
      }

      // update velocity and position
      foreach (Agent agent in agents)
        agent.UpdateVelocityAndPosition();

      iterationCount++;
    }

    public void UpdateParallel()
    {
      // compute agent
      //foreach (Agent agent in agents)
      // Parallel.ForEach(collection, method);
      Parallel.ForEach(agents, agent =>
      {
        //agent.ComputeFlocking(FindNeighbours(agent));
        agent.ComputeStigmergy();
      });

      // update velocity and position
      //foreach (Agent agent in agents)
      Parallel.ForEach(agents, agent =>
      {
        agent.UpdateVelocityAndPosition();
      });

      iterationCount++;
    }

    public List<Agent> FindNeighbours(Agent ag)
    {
      List<Agent> neighbours = new List<Agent>();

      foreach (Agent neighbour in agents)
      {
        Vector3d toNeighbour = neighbour.position - ag.position;
        if (neighbour != ag
          && ag.position.DistanceTo(neighbour.position) < NeighborhoodRadius
          && Vector3d.VectorAngle(ag.velocity, toNeighbour) < AngleOfVision)
        {
          neighbours.Add(neighbour);
        }
      }

      return neighbours;
    }

    public void Output()
    {
      outputValues = new OutputValues(agents.Count);

      //for(int i=0; i< agents.Count; i++)
      Parallel.For(0, agents.Count, i => 
      {
        outputValues.positions[i] = new GH_Point(agents[i].position);
        outputValues.velocities[i] = new GH_Vector(agents[i].velocity);
        outputValues.trails[i] = agents[i].trail;
      });
    }

  }

  public class Agent
  {
    // fields
    public Point3d position;
    public Vector3d velocity;
    public Vector3d desired;
    public AgentSystem agentSystem;
    public double maxSpeed;
    public Polyline trail;
    public int trailLength;
    public int trailFrequency;
    //List<Point3d> pointList;
    //Point3d[] pointArray;
    // constructor
    public Agent(Point3d position, Vector3d velocity)
    {
      //pointList = new List<Point3d>();
      //pointArray = new Point3d[] { new Point3d(0,2,3), new Point3d()};

      this.position = position;
      this.velocity = velocity;
      //desired = velocity;
      maxSpeed = 0.3;
      trail = new Polyline(new Point3d[] { this.position });
      trailLength = 10;
      trailFrequency = 4;
    }

    public Agent(Point3d pos, Vector3d vel, AgentSystem agentSystem) : this(pos, vel)
    {
      this.agentSystem = agentSystem;
    }

    // methods
    public void UpdateVelocityAndPosition()
    {
      // steering
      velocity = 0.97 * velocity + 0.03 * desired;

      if (velocity.Length > maxSpeed)
      {
        velocity.Unitize();
        velocity *= maxSpeed;
      }

      // update position
      position += velocity;

      // update trail
      if (agentSystem.iterationCount % trailFrequency == 0)
        trail.Add(position);

      if (trail.Count > trailLength)
        trail.RemoveAt(0);

      // remove from list
      //for (int i = List.Count - 1; i >= 0; i--)
      //{
      //  check if elemnt is removed
      // list.RemoveAt(i);
      //}
    }

    public void SeekTarget(Point3d target, double intensity)
    {
      Vector3d seek = target - position;
      desired += seek * intensity;
    }

    public Point3d FindCLosestTrailPoint()
    {
      Point3d trailpoint, closest = position;
      double dist, minDist = agentSystem.NeighborhoodRadius;
      int value, maxValue = -1;

      foreach (Agent other in agentSystem.agents)
      {
        if (other == this) continue;

        value = other.trail.ClosestIndex(position + velocity * 3.0);
        trailpoint = other.trail[value];

        Vector3d toTrailPoint = trailpoint - position;
        dist = toTrailPoint.Length;
        if (dist < minDist
          && Vector3d.VectorAngle(toTrailPoint, velocity) < agentSystem.AngleOfVision)
          if (value > maxValue)
          {
            closest = trailpoint;
            minDist = dist;
            maxValue = value;
          }
      }
      return closest;
    }

    public void ComputeStigmergy()
    {
      ResetDesired();
      Containment();
      SeekTarget(FindCLosestTrailPoint(), 10.0);
    }

    public void ComputeFlocking(List<Agent> neighbours)
    {
      ResetDesired();
      Containment();
      Flocking(neighbours);
    }

    public void Flocking(List<Agent> neighbours)
    {
      if (neighbours.Count == 0) return;

      // cohesion
      // find neighbours average position
      Point3d average = new Point3d();

      foreach (Agent neighbour in neighbours)
        average += neighbour.position;

      average /= neighbours.Count;

      // go there
      Vector3d cohesion = average - position;

      // update desired
      desired += cohesion * agentSystem.CohesionStrength;

      // alignment
      Vector3d alignment = Vector3d.Zero;

      foreach (Agent neighbour in neighbours)
        alignment += neighbour.velocity;

      alignment /= neighbours.Count;

      desired += alignment * agentSystem.AlignmentStrength;

      // separation
      Vector3d separation = Vector3d.Zero;

      foreach (Agent neighbour in neighbours)
      {
        // check if neighbour is in separation radius
        double distanceToNeighbour = neighbour.position.DistanceTo(position);
        if (distanceToNeighbour < agentSystem.SeparationRadius)
        {
          Vector3d getaway = position - neighbour.position;
          getaway /= (getaway.Length * distanceToNeighbour);
          separation += getaway;
        }
        // update desired
        desired += separation * agentSystem.SeparationStrength;
      }
    }

    public void Containment()
    {
      if (position.X < 0.0)
        desired += new Vector3d(-position.X, 0, 0);
      else if (position.X > agentSystem.BoundingBoxSize)
        desired += new Vector3d(agentSystem.BoundingBoxSize - position.X, 0, 0);

      if (position.Y < 0.0)
        desired += new Vector3d(0, -position.Y, 0);
      else if (position.Y > agentSystem.BoundingBoxSize)
        desired += new Vector3d(0, agentSystem.BoundingBoxSize - position.Y, 0);

      // make for Z direction


      //desired *= agentSystem.ContainmentStrength;
    }

    public void ResetDesired()
    {
      desired = velocity;
    }
  }

  public double ToRadians(double degrees)
  {
    return degrees * Math.PI / 180.0;
  }
  #endregion
}