using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;



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
  private void RunScript(bool reset, bool go, List<Point3d> P, List<Vector3d> V, double nR, double coS, double alS, double seS, double seR, ref object oP, ref object oV)
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

      // simulaiton update
      agSys.Update();
      Component.ExpireSolution(true);
    }
    // output
    outPos = new List<Point3d>();
    outVel = new List<Vector3d>();

    foreach (Agent agent in agSys.agents)
    {
      outPos.Add(agent.position);
      outVel.Add(agent.velocity);
    }

    oP = outPos;
    oV = outVel;
  }
  #endregion
  #region Additional
  // global variables
  public AgentSystem agSys;
  public List<Point3d> outPos;
  public List<Vector3d> outVel;

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

      BoundingBoxSize = 30.0;
      ContainmentStrength = 1.0;
      MaxSpeed = 0.3;
    }

    // methods
    public void Update()
    {
      // compute
      foreach (Agent agent in agents)
      {
        //List<Agent> neighbours = FindNeighbours(agent);
        //agent.Compute(neighbours);
        agent.Compute(FindNeighbours(agent));
      }

      // update velocity & position
      foreach (Agent agent in agents)
        agent.UpdateVelocityAndPosition();
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

  }

  public class Agent
  {
    // fields
    public Point3d position;
    public Vector3d velocity;
    public Vector3d desired;
    public AgentSystem agentSystem;

    // constructor(s)
    public Agent(Point3d position, Vector3d velocity)
    {
      this.position = position;
      this.velocity = velocity;
    }

    // methods

    public void Compute(List<Agent> neighbours)
    {
      ResetDesired();
      Containment();
      Flocking(neighbours);
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

      // . . . . . separation

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
    }
  }
  #endregion
}