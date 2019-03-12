package org.uma.jmetal.problem.binaryproblem.impl;

import org.uma.jmetal.problem.binaryproblem.BinaryProblem;
import org.uma.jmetal.problem.impl.AbstractGenericProblem;
import org.uma.jmetal.solution.binarysolution.BinarySolution;
import org.uma.jmetal.solution.binarysolution.impl.DefaultBinarySolution;

import java.util.List;

@SuppressWarnings("serial")
public abstract class AbstractBinaryProblem extends AbstractGenericProblem<BinarySolution>
  implements BinaryProblem {

  public abstract List<Integer> getBitsPerVariable() ;

  @Override
  public int getBitsFromVariable(int index) {
    return getBitsPerVariable().get(index) ;
  }
  
  @Override
  public int getTotalNumberOfBits() {
  	int count = 0 ;
  	for (int i = 0; i < this.getNumberOfVariables(); i++) {
  		count += this.getBitsPerVariable().get(i) ;
  	}
  	
  	return count ;
  }

  @Override
  public BinarySolution createSolution() {
    return new DefaultBinarySolution(getBitsPerVariable(), getNumberOfObjectives())  ;
  }
}