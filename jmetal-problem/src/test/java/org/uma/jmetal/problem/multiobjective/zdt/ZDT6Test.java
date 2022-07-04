package org.uma.jmetal.problem.multiobjective.zdt;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;
import org.uma.jmetal.problem.doubleproblem.DoubleProblem;
import org.uma.jmetal.solution.doublesolution.DoubleSolution;

class ZDT6Test {
  @Test
  void constructorMustCreateAValidInstanceUsingTheDefaultConstructor() {
    int defaultNumberOfVariables = 10 ;
    DoubleProblem problem = new ZDT6() ;

    assertThat(problem.getNumberOfVariables()).isEqualTo(defaultNumberOfVariables) ;
    assertThat(problem.getNumberOfObjectives()).isEqualTo(2) ;
    assertThat(problem.getNumberOfConstraints()).isZero() ;
    assertThat(problem.getName()).isEqualTo("ZDT6") ;

    assertThat(problem.getVariableBounds().get(0).getLowerBound()).isZero() ;
    assertThat(problem.getVariableBounds().get(0).getUpperBound()).isEqualTo(1) ;
    assertThat(problem.getVariableBounds().get(problem.getNumberOfVariables()-1).getLowerBound()).isZero() ;
    assertThat(problem.getVariableBounds().get(problem.getNumberOfVariables()-1).getUpperBound()).isEqualTo(1) ;
  }

  @Test
  void constructorMustCreateAValidInstanceWhenIndicatingTheNumberOVariables() {
    int numberOfVariables = 20 ;
    DoubleProblem problem = new ZDT6(numberOfVariables) ;

    assertThat(problem.getNumberOfVariables()).isEqualTo(numberOfVariables) ;
    assertThat(problem.getNumberOfObjectives()).isEqualTo(2) ;
    assertThat(problem.getNumberOfConstraints()).isZero() ;
    assertThat(problem.getName()).isEqualTo("ZDT6") ;

    assertThat(problem.getVariableBounds().get(0).getLowerBound()).isZero() ;
    assertThat(problem.getVariableBounds().get(0).getUpperBound()).isEqualTo(1) ;
    assertThat(problem.getVariableBounds().get(problem.getNumberOfVariables()-1).getLowerBound()).isZero() ;
    assertThat(problem.getVariableBounds().get(problem.getNumberOfVariables()-1).getUpperBound()).isEqualTo(1) ;
  }

  @Test
  void createSolutionGeneratesAValidSolution() {
    int numberOfVariables = 12 ;

    DoubleProblem problem = new ZDT6(numberOfVariables) ;
    DoubleSolution solution = problem.createSolution() ;

    assertThat(solution).isNotNull() ;
    assertThat(solution.variables()).hasSize(numberOfVariables) ;
    assertThat(solution.objectives()).hasSize(2) ;
    assertThat(solution.constraints()).isEmpty() ;
  }
}