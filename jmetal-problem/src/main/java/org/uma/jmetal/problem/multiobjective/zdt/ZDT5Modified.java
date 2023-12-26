//  ZDT5.java
//
//  Author:
//       Antonio J. Nebro <antonio@lcc.uma.es>
//       Juan J. Durillo <durillo@lcc.uma.es>
//
//  Copyright (c) 2011 Antonio J. Nebro, Juan J. Durillo
//

//
//  Modify as a AbstractDoubleProblem problem
//

package org.uma.jmetal.problem.multiobjective.zdt;

import org.uma.jmetal.problem.binaryproblem.impl.AbstractBinaryProblem;
import org.uma.jmetal.solution.binarysolution.BinarySolution;
import org.uma.jmetal.solution.doublesolution.DoubleSolution;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/** Class representing problem ZDT5 */
public class ZDT5Modified extends ZDT1 {
  private int bitsPerVariable = 5;
  private int numberOfVariables;
  private int NumberOfVariablesActual = 11;

  /** Creates a default instance of problem ZDT5 (11 decision variables, but need 80 bits to represent) */
  public ZDT5Modified() {
    this(80);
  }

  /**
   * Creates a instance of problem ZDT5
   *
   * @param numberOfVariables Number of variables.
   */
  public ZDT5Modified(Integer numberOfVariables) {
    super(30 + (numberOfVariables - 1) * 5);
    this.NumberOfVariablesActual = numberOfVariables;
    name("ZDT5");
  }

  /**
   * Creates a instance of problem ZDT5
   *
   * @param numberOfVariables Number of variables.
   * @param bitsPerVariable Number of bits for representing variables.
   */
  public ZDT5Modified(Integer numberOfVariables, Integer bitsPerVariable) {
    super(30 + (numberOfVariables - 1) * bitsPerVariable);
    this.NumberOfVariablesActual = numberOfVariables;
    this.bitsPerVariable = bitsPerVariable;
    name("ZDT5");
  }

  public Integer numberOfBitsPerVariable() {
    return bitsPerVariable;
  }

  /** Evaluate() method */
  public DoubleSolution evaluate(DoubleSolution solution) {
    double[] f = new double[solution.objectives().length];
    f[0] = 1 + u(solution.variables().subList(0, 30));
    double g = evalG(solution);
    double h = evalH(f[0], g);
    f[1] = h * g;

    f[0] = ((double) (f[0] - 1)) / (31 - 1);
    f[1] = (f[1] - (((double) NumberOfVariablesActual - 1) / 31)) / (((double) NumberOfVariablesActual - 1) - (((double) NumberOfVariablesActual - 1) / 31));

    solution.objectives()[0] = f[0];
    solution.objectives()[1] = f[1];

    return solution;
  }

  /**
   * Returns the value of the ZDT5 function G.
   *
   * @param solution The solution.
   */
  public double evalG(DoubleSolution solution) {
    double res = 0.0;
    for (int i = 0; i < NumberOfVariablesActual - 1; i++) {
      res += evalV(u(solution.variables().subList(30 + bitsPerVariable * i, 30 + bitsPerVariable * (i + 1))));
    }

    return res;
  }

  /**
   * Returns the value of the ZDT5 function V.
   *
   * @param value The parameter of V function.
   */
  public double evalV(double value) {
    if (value < (double) bitsPerVariable) {
      return 2.0 + value;
    } else {
      return 1.0;
    }
  }

  /**
   * Returns the value of the ZDT5 function H.
   *
   * @param f First argument of the function H.
   * @param g Second argument of the function H.
   */
  public double evalH(double f, double g) {
    return 1 / f;
  }

  /**
   * Returns the u value defined in ZDT5 for a solution.
   *
   * @param bitset A bitset variable
   */
  private double u(List<Double> bitset) {
    return bitset.stream().reduce(Double::sum).orElse(0.0);
  }
}
