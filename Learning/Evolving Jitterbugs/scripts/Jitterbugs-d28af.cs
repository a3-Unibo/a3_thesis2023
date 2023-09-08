using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

using System.IO;


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public abstract class Script_Instance_d28af : GH_ScriptInstance
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
  private void RunScript(bool go, bool reset, string path, int iter, int episodes, double mR, double mA, int nB, Point3d P, Point3d T, List<Polyline> O, ref object o_P, ref object o_V, ref object bFitness, ref object gen, ref object iters)
  {

    // reset & initialize conditions
    if (reset || bugSimulation == null)
    {
      // initialize simulation
      bugSimulation = new BugSystem(nB, P, mA, mR, iter, episodes, path)
      {
        target = T,
        obstacles = O
      };
      return;
    }

    // update variables
    // episodes can be increased if the fitness is not satisfactory
    if (!go)
    {
      bugSimulation.maxEpisodes = episodes;
    }

    // simulate current iteration
    if (go && !bugSimulation.EndEpisode())
    {
      bugSimulation.Update();
      Component.ExpireSolution(true);
    }

    // check if episode is finished
    if (bugSimulation.EndEpisode())
    {
      // compute fitness and find best bug
      bugSimulation.ComputeFitnesses();
      bugSimulation.FindBestBug();

      // record episode
      bugSimulation.RecordEpisode();

      // if simulation is not ended prepare next episode
      if (!bugSimulation.EndSimulation()) bugSimulation.PrepareNextEpisode();
    }

    // output
    // output variables
    List<GH_Point> positions = new List<GH_Point>();
    List<GH_Vector> velocities = new List<GH_Vector>();

    foreach (JitterBug bug in bugSimulation.bugs)
    {
      positions.Add(new GH_Point(bug.position));
      velocities.Add(new GH_Vector(bug.velocity));
    }

    o_P = positions;
    o_V = velocities;
    bFitness = bugSimulation.fitnesses;
    gen = bugSimulation.currEpisode;
    iters = bugSimulation.currIteration;

  }
  #endregion
  #region Additional

  BugSystem bugSimulation;

  public class BugSystem
  {
    public JitterBug[] bugs;
    public List<Polyline> obstacles;
    public List<GH_Number> fitnesses;
    public JitterBug bestBug;
    public Point3d target;
    public int maxIterations;
    public int currIteration;
    public int maxEpisodes;
    public int currEpisode;
    public double threshold;
    public double totalFitness;
    public Random rnd;
    public int seed;
    public string bugExport;
    public string brainExport;
    public string path;

    public BugSystem(int nBugs, Point3d basePoint, double maxAngle, double mutationRate, int maxIterations, int maxEpisodes, string path)
    {
      bugs = new JitterBug[nBugs];
      this.maxIterations = maxIterations;
      this.maxEpisodes = maxEpisodes;
      currEpisode = 0;
      currIteration = 0;
      threshold = 1.0;
      seed = DateTime.Now.Hour + DateTime.Now.Minute + DateTime.Now.Second;
      rnd = new Random(seed);

      for (int i = 0; i < nBugs; i++)
        bugs[i] = new JitterBug(this, basePoint, mutationRate);

      fitnesses = new List<GH_Number>();
      for (int i = 0; i < nBugs; i++) fitnesses.Add(new GH_Number(0));

      this.path = path;

      InitDirectory();
    }

    public void Update()
    {
      foreach (JitterBug bug in bugs) bug.Update();
      currIteration++;
    }
    public bool Genocide()
    {
      bool genocide = true;
      foreach (JitterBug bug in bugs)
        if (!bug.dead)
        {
          genocide = false;
          break;
        }

      return genocide;
    }

    public bool EndEpisode()
    {
      return (currIteration >= maxIterations-1) || Genocide();
    }

    public bool EndSimulation()
    {
      return currEpisode >= maxEpisodes - 1;
    }

    public void InitDirectory()
    {
      string subPath = DateTime.Now.Year.ToString() + DateTime.Now.Month.ToString("00") + DateTime.Now.Day.ToString("00") + "_" +
        DateTime.Now.Hour.ToString("00") + DateTime.Now.Minute.ToString("00") + "\\";
      path += subPath;

      if (!Directory.Exists(path)) Directory.CreateDirectory(path);
      return;
    }

    public void RecordEpisode()
    {
      ConvertToString();
      WriteToFile(path);
    }

    public void ConvertToString()
    {
      bugExport = "";
      for (int i = 0; i < bugs.Length; i++)
      {
        JitterBug bug = bugs[i];
        bugExport += string.Format("{0},{1},{2},{3}", bug.position.X, bug.position.Y, bug.nMoves + 1, bug.fitness);
        if (i < bugs.Length - 1) bugExport += "\n";
      }

      brainExport = bestBug.nMoves.ToString() + "\n";
      for (int i = 0; i < bestBug.brain.actions.Length; i++)
      {
        brainExport += bestBug.brain.actions[i].ToString();
        if (i < bestBug.brain.actions.Length - 1) brainExport += "\n";
      }
    }

    public void WriteToFile(string path)
    {
      string epPath = path + "episode_" + currEpisode.ToString() + ".txt";
      string brainPath = path + "brain_" + currEpisode.ToString() + ".txt";
      File.WriteAllText(epPath, bugExport);
      File.WriteAllText(brainPath, brainExport);
    }

    public void ResetEpisode()
    {
      currIteration = 0;
      currEpisode++;
      foreach (JitterBug bug in bugs) bug.Reset();
    }

    public void PrepareNextEpisode()
    {
      // select parents and make offspring
      NaturalSelection();

      // mutate children bugs
      MutateBugs();

      // reset episode variables
      ResetEpisode();
    }

    public void ComputeFitnesses()
    {
      fitnesses.Clear();
      foreach (JitterBug b in bugs)
      {
        b.ComputeFitness();
        fitnesses.Add(new GH_Number(b.fitness));
      }
    }

    public void ComputeTotalFitness()
    {
      totalFitness = 0;
      foreach (JitterBug b in bugs) totalFitness += b.fitness;
    }

    public void FindBestBug()
    {
      int best = 0;
      double maxScore = bugs[0].fitness;

      for (int i = 1; i < bugs.Length; i++)
      {
        if (bugs[i].fitness > maxScore)
        {
          maxScore = bugs[i].fitness;
          best = i;
        }
      }

      bestBug = bugs[best].Clone();
      bestBug.nMoves = bugs[best].nMoves;
    }

    public JitterBug GetParent()
    {
      double scoreThres = rnd.NextDouble() * totalFitness;

      double runningSum = 0;

      for (int i = 0; i < bugs.Length; i++)
      {
        runningSum += bugs[i].fitness;
        if (runningSum > scoreThres)
        {
          return bugs[i];
        }
      }

      // this should never happen
      return null;
    }

    public void NaturalSelection()
    {
      JitterBug[] newBugs = new JitterBug[bugs.Length];

      // compute total fitness
      ComputeTotalFitness();

      // reset bestBug nMoves
      bestBug.nMoves = -1;

      // insert best bug at index 0
      newBugs[0] = bestBug;

      for (int i = 1; i < newBugs.Length; i++)
      {
        // select parent based on fitness
        JitterBug parent = GetParent();

        // new bug is the parent's clone
        newBugs[i] = parent.Clone();
      }

      bugs = (JitterBug[])newBugs.Clone();
    }

    public void MutateBugs()
    {
      for (int i = 0; i < bugs.Length; i++)
        bugs[i].brain.Mutate();
    }
  }

  /// <summary>
  /// JitterBug class
  /// </summary>
  public class JitterBug
  {
    public BugSystem bugsim;
    public Point3d position;
    public Point3d initPos;
    public Vector3d velocity;
    public Vector3d acceleration;
    public bool arrived;
    public bool dead;
    public double fitness;
    public double maxSpeed;
    static int id;
    public int seed;
    public int nMoves;
    public Brain brain;

    public JitterBug(BugSystem bugsim, Point3d position, double mutationRate)
    {
      this.bugsim = bugsim;
      this.position = position;
      initPos = position;
      velocity = Vector3d.Zero;
      arrived = false;
      dead = false;
      fitness = 0.0;
      maxSpeed = 1.0;
      nMoves = -1;
      seed = id;
      brain = new Brain(seed, bugsim.maxIterations, mutationRate);
      id++;
    }

    public JitterBug Clone()
    {
      JitterBug clone = new JitterBug(bugsim, initPos, brain.mutationRate);
      clone.brain = brain.Clone(clone.seed);
      return clone;
    }

    public JitterBug CloneWithActions(Vector3d[] actions)
    {
      JitterBug clone = new JitterBug(bugsim, initPos, brain.mutationRate);
      // clone actions only to preserve new seed for random
      clone.brain.actions = (Vector3d[])actions.Clone();
      return clone;
    }

    public void Reset()
    {
      position = initPos;
      dead = false;
      arrived = false;
      nMoves = -1;
    }

    public void Update()
    {
      if (dead || arrived) return;
      nMoves++;
      acceleration = brain.actions[nMoves];
      Move();
      CheckBoundaries();
      Arrived();
    }

    private void CheckBoundaries()
    {
      foreach (Polyline obstacle in bugsim.obstacles)
      {
        if (obstacle.ClosestPoint(position).DistanceToSquared(position) < 0.5)
        {
          dead = true;
          break;
        }
      }
    }

    public void Move()
    {
      velocity += acceleration;
      if (velocity.Length > maxSpeed)
      {
        velocity.Unitize();
        velocity *= maxSpeed;
      }
      position += velocity;
    }

    public void Arrived()
    {
      arrived = position.DistanceToSquared(bugsim.target) < bugsim.threshold;
    }

    public void ComputeFitness()
    {
      int penalty = 0;
      if (dead) penalty = 10;
      else if (arrived) penalty = -5;
      fitness = 1 / (position.DistanceToSquared(bugsim.target) + 0.001 + (nMoves + penalty) * 0.1);

      // try this other fitness function and see if it works better or worse

      //double reward = 1;
      //if (dead) reward = 0.1;
      //else if (arrived) reward = 2;
      //fitness = (100000 * reward) / ((position.DistanceToSquared(bugsim.target) + 0.001) * nMoves * nMoves);
    }
  }

  /// <summary>
  /// Brain class
  /// </summary>
  public class Brain
  {
    public Vector3d[] actions;
    public double mutationRate;
    public Random rnd;
    public int seed;
    public int step;

    public Brain(int seed, int nActions, double mutationRate)
    {
      actions = new Vector3d[nActions];
      this.seed = seed;
      this.mutationRate = mutationRate;
      step = 0;
      rnd = new Random(seed);
      Vector3d vel = Vector3d.YAxis;

      RandomizeActions();
    }

    public void RandomizeActions()
    {
      Vector3d vel;

      for (int i = 0; i < actions.Length; i++)
      {
        vel = Vector3d.YAxis;
        vel.Rotate(rnd.NextDouble() * Math.PI * 2, Vector3d.ZAxis);
        actions[i] = vel;
      }
    }

    public Brain Clone(int seed)
    {
      Brain clone = new Brain(seed, actions.Length, mutationRate);

      clone.actions = (Vector3d[])actions.Clone();
      return clone;
    }

    public void Mutate()
    {
      for (int i = 0; i < actions.Length; i++)
      {
        if (rnd.NextDouble() < mutationRate)
        {
          Vector3d newAction = Vector3d.YAxis;
          newAction.Rotate(rnd.NextDouble() * Math.PI * 2, Vector3d.ZAxis);
          actions[i] = newAction;
        }
      }
    }


  }
  #endregion
}