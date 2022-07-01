package org.uma.jmetal.example.operator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.uma.jmetal.lab.visualization.plot.PlotFront;
import org.uma.jmetal.lab.visualization.plot.impl.PlotSmile;
import org.uma.jmetal.operator.crossover.CrossoverOperator;
import org.uma.jmetal.operator.crossover.impl.IntegerSBXCrossover;
import org.uma.jmetal.problem.integerproblem.IntegerProblem;
import org.uma.jmetal.problem.multiobjective.NMMin;
import org.uma.jmetal.solution.integersolution.IntegerSolution;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.bounds.Bounds;
import org.uma.jmetal.util.comparator.IntegerVariableComparator;

/**
 * @author Antonio J. Nebro
 * @version 1.0
 *
 * This class is intended to verify the working of the SBX crossover operator for Integer encoding
 *
 * A figure depicting the values obtained when generating 100000 points, a granularity of 200, and a number
 * of different distribution index values (5, 10, 20) can be found here:
 * <a href="https://github.com/jMetal/jMetal/blob/master/figures/integerPolynomialMutation.png">
 * Polynomial mutation (Integer) </a>
 */
public class IntegerSBXCrossoverExample {
  /**
   * Program to generate data representing the distribution of points generated by a SBX
   * crossover operator. The parameters to be introduced by the command line are:
   * - numberOfSolutions: number of solutions to generate
   * - granularity: number of subdivisions to be considered.
   * - distributionIndex: distribution index of the polynomial mutation operator
   * - outputFile: file containing the results
   *
   * @param args Command line arguments
   */
  public static void main(String[] args) {
    int numberOfPoints ;
    int granularity ;
    double distributionIndex ;

    if (args.length !=3) {
      JMetalLogger.logger.info("Usage: numberOfSolutions granularity distributionIndex") ;
      JMetalLogger.logger.info("Using default parameters") ;

      numberOfPoints = 10000 ;
      granularity = 100 ;
      distributionIndex = 10.0 ;
    } else {
      numberOfPoints = Integer.parseInt(args[0]);
      granularity = Integer.parseInt(args[1]);
      distributionIndex = Double.parseDouble(args[2]);
    }

    IntegerProblem problem ;

    problem = new NMMin(1, -50, 50, -100, 100) ;
    CrossoverOperator<IntegerSolution> crossover = new IntegerSBXCrossover(1.0, distributionIndex) ;

    IntegerSolution solution1 = problem.createSolution() ;
    IntegerSolution solution2 = problem.createSolution() ;
    solution1.variables().set(0, -50);
    solution2.variables().set(0, 50);
    List<IntegerSolution> parents = Arrays.asList(solution1, solution2) ;

    List<IntegerSolution> population = new ArrayList<>(numberOfPoints) ;
    for (int i = 0; i < numberOfPoints ; i++) {
      List<IntegerSolution> solutions = (List<IntegerSolution>) crossover.execute(parents);
      population.add(solutions.get(0)) ;
      population.add(solutions.get(1)) ;
    }

    Collections.sort(population, new IntegerVariableComparator()) ;

    double[][] classifier = classify(population, problem, granularity);

    PlotFront plot = new PlotSmile(classifier) ;
    plot.plot();
  }

  private static double[][] classify(List<IntegerSolution> solutions, IntegerProblem problem, int granularity) {
    Bounds<Integer> bounds = problem.getVariableBounds().get(0);
    double grain = (bounds.getUpperBound() - bounds.getLowerBound()) / granularity ;
    double[][] classifier = new double[granularity][] ;
    for (int i = 0 ; i < granularity; i++) {
      classifier[i] = new double[2] ;
      classifier[i][0] = bounds.getLowerBound() + i * grain ;
      classifier[i][1] = 0 ;
    }

    for (IntegerSolution solution : solutions) {
      boolean found = false ;
      int index = 0 ;
      while (!found) {
        if (solution.variables().get(0) <= classifier[index][0]) {
          classifier[index][1] ++ ;
          found = true ;
        } else {
          if (index == (granularity - 1)) {
            classifier[index][1] ++ ;
            found = true ;
          } else {
            index++;
          }
        }
      }
    }

    return classifier ;
  }
}
