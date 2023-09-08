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
public abstract class Script_Instance_de5b1 : GH_ScriptInstance
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
  private void RunScript(bool reset, bool go, int nData, double LR, ref object Points, ref object inf, ref object line, ref object Perc_line, ref object Weights)
  {
    // . . . . 0.initialize
    if (reset || pTron == null)
    {
      count = 0;
      training = GenerateTrainingData(nData);
      pTron = new Perceptron(training[0].inputs.Length, LR);
      inferences = new List<int>();
    }

    // . . . . 1.update

    // update live variables


    if (go)
    {
      // update simulation
      pTron.Train(training[count].inputs, training[count].answer);
      count = (count + 1) % training.Length;

      // make inferences
      inferences.Clear();
      for (int i = 0; i < count; i++)
        inferences.Add(pTron.FeedForward(training[i].inputs));

      Component.ExpireSolution(true);
    }

    // . . . . 2.output
    line = new Line(-50, Function(-50), 0, 50, Function(50), 0);
    Perc_line = pTron.ToLine(-50, 50);
    Weights = pTron.weights;
    inf = inferences;

    List<GH_Point> points = new List<GH_Point>();
    for (int i = 0; i < count; i++)
      points.Add(new GH_Point(
        new Point3d(training[i].inputs[0],training[i].inputs[1], 0)));
    
    Points = points;
  }
  #endregion
  #region Additional
  // global variables
  Perceptron pTron;
  TrainingData[] training;
  int count;
  List<int> inferences;

  // simulation classes
  public class Perceptron
  {
    public double[] weights;
    public double learningRate;
    public double error;
    Random rnd;

    public Perceptron(int nWeights, double learningRate)
    {
      weights = new double[nWeights];
      this.learningRate = learningRate;
      rnd = new Random();
      // initialize random weights
      for (int i = 0; i < nWeights; i++)
        weights[i] = rnd.NextDouble() * 2 - 1;
    }

    public int Activate(double sum)
    {
      if (sum > 0) return 1;
      else return 0;
    }

    public int FeedForward(double[] inputs)
    {
      double sum = 0;
      for (int i = 0; i < weights.Length; i++)
        sum += inputs[i] * weights[i];

      return Activate(sum);
    }

    public void Train(double[] inputs, int answer)
    {
      int guess = FeedForward(inputs);
      error = answer - guess;
      for(int i=0; i< weights.Length; i++)
        weights[i] += error * inputs[i] * learningRate;
    }

    public Line ToLine(double x0, double x1)
    {
      return new Line(x0, -(x0 * weights[0] + weights[2]) / (weights[1]+0.000001), 0,
        x1, -(x1 * weights[0] + weights[2]) / (weights[1] + 0.000001), 0);
    }
  }

  // helper classes

  public class TrainingData
  {
    public double[] inputs;
    public int answer;

    public TrainingData(double x, double y, int a)
    {
      inputs = new double[3];

      inputs[0] = x;
      inputs[1] = y;
      inputs[2] = 1; // bias

      answer = a;
    }
  }

  // helper methods

  public TrainingData[] GenerateTrainingData(int nData)
  {
    Random rnd = new Random(DateTime.Now.Millisecond);
    TrainingData[] training = new TrainingData[nData];
    // make training points
    for (int i = 0; i < training.Length; i++)
    {
      double x = rnd.NextDouble() * 100 - 50;
      double y = rnd.NextDouble() * 100 - 50;
      int answer = 1;
      if (y < Function(x)) answer = -1;
      training[i] = new TrainingData(x, y, answer);
    }

    return training;
  }

  public double Function(double x)
  {
    return 2 * x + 1;
  }
  #endregion
}