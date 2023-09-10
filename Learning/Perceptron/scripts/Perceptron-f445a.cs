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
public abstract class Script_Instance_f445a : GH_ScriptInstance
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
  private void RunScript(bool reset, bool go, int nData, double LR, ref object Points, ref object inf, ref object line, ref object Perceptron_line, ref object Weights, ref object error)
  {
    // reset & setup
    if (reset || perceptron == null)
    {
      count = 0;
      trainingData = GenerateTrainingData(nData);
      perceptron = new Perceptron(trainingData[0].inputs.Length, LR);
      inferences = new List<int>();
    }

    // simulation

    if (go)
    {
      // train Perceptron one point at a time (for animation)
      perceptron.Train(trainingData[count].inputs, trainingData[count].label);
      count = (count + 1) % trainingData.Length;

      // make inferences
      inferences.Clear();
      for (int i = 0; i < count; i++)
        inferences.Add(perceptron.FeedForward(trainingData[i].inputs));

      Component.ExpireSolution(true);
    }

    // output
    line = new Line(-50, Function(-50), 0, 50, Function(50), 0);
    Perceptron_line = perceptron.ToLine(-50, 50);
    Weights = perceptron.weights;
    inf = inferences;
    error = perceptron.error;

    List<GH_Point> points = new List<GH_Point>();
    for (int i = 0; i < count; i++)
    {
      points.Add(new GH_Point(new Point3d(trainingData[i].inputs[0], trainingData[i].inputs[1], 0)));
    }

    Points = points;
  }
  #endregion
  #region Additional

  Perceptron perceptron;
  TrainingData[] trainingData;
  int count;
  List<int> inferences;

  public double Function(double x)
  {
    return 2 * x + 1;
  }

  public TrainingData[] GenerateTrainingData(int nData)
  {
    Random rnd = new Random(DateTime.Now.Millisecond);
    TrainingData[] trainingData = new TrainingData[nData];
    // make training points
    for (int i = 0; i < trainingData.Length; i++)
    {
      Point2d point = new Point2d(rnd.NextDouble() * 100 - 50, rnd.NextDouble() * 100 - 50);
      int label = 1;
      if (point.Y < Function(point.X)) label = -1;
      trainingData[i] = new TrainingData(point.X, point.Y, label);
    }

    return trainingData;
  }

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

      // initialize with random weights
      for (int i = 0; i < nWeights; i++)
        weights[i] = rnd.NextDouble() * 2 - 1;
    }

    public int FeedForward(double[] inputs)
    {
      double sum = 0;
      for (int i = 0; i < weights.Length; i++)
        sum += inputs[i] * weights[i];

      return Activate(sum);
    }

    public int Activate(double sum)
    {
      if (sum > 0) return 1;
      else return 0;
    }

    public void Train(double[] inputs, int label)
    {
      int prediction = FeedForward(inputs);
      error = label - prediction;
      for (int i = 0; i < weights.Length; i++)
        weights[i] += error * inputs[i] * learningRate;
    }

    public Line ToLine(double x0, double x1)
    {
      // Formula is weights[0]*x + weights[1]*y + weights[2] = 0
      return new Line(x0, -(x0 * weights[0] + weights[2]) / weights[1], 0, x1, -(x1 * weights[0] + weights[2]) / weights[1], 0);
    }
  }

  public class TrainingData
  {
    public double[] inputs;
    public int label;

    public TrainingData(double x, double y, int label)
    {
      inputs = new double[3];
      inputs[0] = x;
      inputs[1] = y;
      inputs[2] = 1; // bias

      this.label = label;
    }
  }
  #endregion
}