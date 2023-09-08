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
using Plankton;


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public abstract class Script_Instance_82770 : GH_ScriptInstance
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
    // initialize
    if (agSys == null || reset)
      agSys = new AgentSystem(P, V);

    // update
    if (go)
    {
      // update live variables
      agSys.NeighborhoodRadius = nR;
      agSys.CohesionStrength = coS;
      agSys.AlignmentStrength = alS;
      agSys.SeparationStrength = seS;
      agSys.SeparationRadius = seR;
      agSys.AngleOfVision = ToRadians(aoV);

      // simulation update
      switch (uT)
      {
        case 0:
          agSys.Update();
          break;
        case 1:
          agSys.UpdateParallel();
          break;
        case 2: // RTree
          agSys.UpdateRTree();
          break;
        default:
          goto case 2;
      }

      Component.ExpireSolution(true);
    }
    // output
    //outPos = new List<Point3d>();
    //outVel = new List<Vector3d>();
    //outTrails = new List<Polyline>();

    //foreach (Agent agent in agSys.agents)
    //{
    //  outPos.Add(agent.position);
    //  outVel.Add(agent.velocity);
    //  outTrails.Add(agent.trail);
    //}

    agSys.Output();

    oP = agSys.outputValues.positions; //outPos;
    oV = agSys.outputValues.velocities;//outVel;
    oT = agSys.outputValues.trails; //outTrails;
  }
  #endregion
  #region Additional
  // global variables
  public AgentSystem agSys;
  //public List<Point3d> outPos;
  //public List<Vector3d> outVel;
  //public List<Polyline> outTrails;

  // custom classes
  public class AgentSystem
  {
    // fields
    public List<Agent> agents;
    public double NeighborhoodRadius;
    public double CohesionStrength;
    public double AlignmentStrength;
    public double SeparationStrength;
    public double SeparationRadius;
    public double BoundingBoxSize;
    public double ContainmentStrength;
    public double MaxSpeed;
    public double AngleOfVision;
    public int iterationCount;
    public OutputValues outputValues;

    // constructor(s)
    public AgentSystem(List<Point3d> points, List<Vector3d> vectors)
    {
      agents = new List<Agent>();

      for (int i = 0; i < points.Count; i++)
      {
        Agent ag = new Agent(points[i], vectors[i]);
        ag.agentSystem = this;
        agents.Add(ag);
      }

      iterationCount = 0;
      BoundingBoxSize = 30.0;
      ContainmentStrength = 1.0;
      MaxSpeed = 0.3;
    }

    // methods
    public void UpdateRTree()
    {
      // build RTree
      RTree PointsRTree = new RTree();
      for (int i = 0; i < agents.Count; i++)
        PointsRTree.Insert(agents[i].position, i);

      // find neighbours & call compute
      foreach (Agent agent in agents)
      {
        List<Agent> neighbours = new List<Agent>();

        PointsRTree.Search(new Sphere(agent.position, NeighborhoodRadius), (sender, args) =>
        {
          if (agents[args.Id] != agent)
          {
            Vector3d toNeighbour = agents[args.Id].position - agent.position;

            if (Vector3d.VectorAngle(toNeighbour, agent.velocity) < AngleOfVision)
              neighbours.Add(agents[args.Id]);
          }
        });

        agent.ComputeFlocking(neighbours);
      }

      // update velocity and position
      Parallel.ForEach(agents, agent =>
      {
        agent.UpdateVelocityAndPosition();
      });

      iterationCount++;

    }

    public void UpdateParallel()
    {
      // compute
      Parallel.ForEach(agents, agent =>
      {
        agent.ComputeStigmergy();
      });

      //foreach (Agent agent in agents)
      //{
      //  //List<Agent> neighbours = FindNeighbours(agent);
      //  //agent.Compute(neighbours);
      //  agent.Compute(FindNeighbours(agent));
      //}

      // update velocity & position
      //foreach (Agent agent in agents)
      Parallel.ForEach(agents, agent =>
      {
        agent.UpdateVelocityAndPosition();
      });

      iterationCount++;
    }

    public void Update()
    {
      // compute
      foreach (Agent agent in agents)
      {
        //List<Agent> neighbours = FindNeighbours(agent);
        //agent.Compute(neighbours);
        agent.ComputeFlocking(FindNeighbours(agent));
      }

      // update velocity & position
      foreach (Agent agent in agents)
        agent.UpdateVelocityAndPosition();

      iterationCount++;
    }

    public List<Agent> FindNeighbours(Agent ag)
    {
      List<Agent> neighbours = new List<Agent>();
      foreach (Agent neighbour in agents)
      {
        if (neighbour != ag && ag.position.DistanceTo(neighbour.position) < NeighborhoodRadius)
        {
          neighbours.Add(neighbour);
        }
      }

      return neighbours;
    }

    public void Output()
    {
      outputValues = new OutputValues(agents.Count);

      // for(int i=0; i< agents.Count; i++)
      Parallel.For(0, agents.Count, i =>
      {
        outputValues.positions[i] = new GH_Point(agents[i].position);
        outputValues.velocities[i] = new GH_Vector(agents[i].velocity);
        outputValues.trails[i] = agents[i].trail;
      });

    }

  }

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

  public class Agent
  {
    // fields
    public Point3d position;
    public Vector3d velocity;
    public Vector3d desired;
    public Polyline trail;
    public int trailLength;
    public AgentSystem agentSystem;

    // constructor(s)
    public Agent(Point3d position, Vector3d velocity)
    {
      this.position = position;
      this.velocity = velocity;
      trail = new Polyline(new Point3d[] { this.position });
      trailLength = 10;
      //double[] values = new double[3];
      //values[2] = 1.5;
    }

    // methods

    public void ComputeFlocking(List<Agent> neighbours)
    {
      ResetDesired();
      Containment();
      Flocking(neighbours);
    }

    public void ComputeStigmergy()
    {
      ResetDesired();
      Containment();
      SeekTarget(FindClosestTrailPoint(), 10.0);
    }

    public void SeekTarget(Point3d target, double intensity)
    {
      Vector3d seek = target - position;

      desired += seek * intensity;
    }

    public Point3d FindClosestTrailPoint()
    {
      Point3d trailPoint, closest = position;
      double dist, minDist = agentSystem.NeighborhoodRadius;
      int value, maxValue = -1;

      foreach (Agent other in agentSystem.agents)
      {
        if (other == this) continue;

        value = other.trail.ClosestIndex(position + velocity * 3.0);
        trailPoint = other.trail[value];
        //trailPoint = other.trail.ClosestPoint(position + velocity * 3.0);
        Vector3d toTrailPoint = trailPoint - position;
        dist = toTrailPoint.Length;
        if (dist < minDist && Vector3d.VectorAngle(toTrailPoint, velocity) < agentSystem.AngleOfVision)
          if (value > maxValue)
          {
            closest = trailPoint;
            minDist = dist;
            maxValue = value;
          }
      }

      return closest;
    }

    public void Flocking(List<Agent> neighbours)
    {
      if (neighbours.Count == 0) return;

      // . . . . . cohesion

      // find neighbours average
      Point3d average = new Point3d();

      foreach (Agent neighbour in neighbours)
        average += neighbour.position;

      average /= neighbours.Count;

      // go there
      Vector3d cohesion = average - position;

      // update desired
      desired += cohesion * agentSystem.CohesionStrength;

      // . . . . . alignment
      Vector3d alignment = Vector3d.Zero;

      foreach (Agent neighbour in neighbours)
        alignment += neighbour.velocity;

      alignment /= neighbours.Count;

      desired += alignment * agentSystem.AlignmentStrength;
      // . . . . . separation
      Vector3d separation = Vector3d.Zero;

      foreach (Agent neighbour in neighbours)
      {
        // check if in separation radius
        double distanceToNeighbour = position.DistanceTo(neighbour.position);
        if (distanceToNeighbour < agentSystem.SeparationRadius)
        {
          Vector3d getaway = position - neighbour.position;
          getaway /= (getaway.Length * distanceToNeighbour);
          separation += getaway;
        }
      }

      desired += separation * agentSystem.SeparationStrength;
    }

    public void ResetDesired()
    {
      desired = velocity;
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

      if (position.Z < 0.0)
        desired += new Vector3d(0, 0, -position.Z);
      else if (position.Z > agentSystem.BoundingBoxSize)
        desired += new Vector3d(0, 0, agentSystem.BoundingBoxSize - position.Z);
    }
    public void UpdateVelocityAndPosition()
    {
      // steering
      velocity = 0.97 * velocity + 0.03 * desired;

      if (velocity.Length > agentSystem.MaxSpeed)
      {
        velocity.Unitize();
        velocity *= agentSystem.MaxSpeed;
      }

      // update position
      position += velocity;

      // update trail
      if (agentSystem.iterationCount % 4 == 0)
        trail.Add(position);

      if (trail.Count > trailLength)
        trail.RemoveAt(0);

    }
  }

  public double ToRadians(double degrees)
  {
    return degrees * Math.PI / 180.0;
  }
  #endregion
}