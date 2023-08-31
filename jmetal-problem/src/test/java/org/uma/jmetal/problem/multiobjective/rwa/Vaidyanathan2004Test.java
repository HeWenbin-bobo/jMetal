package org.uma.jmetal.problem.multiobjective.rwa;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.uma.jmetal.problem.doubleproblem.DoubleProblem;

@DisplayName("Class Vaidyanathan2004")
class Vaidyanathan2004Test {
  private DoubleProblem problem;

  @BeforeEach
  void setup() {
    problem = new Vaidyanathan2004();
  }

  @DisplayName("Its main properties are:")
  @Nested
  class MainProperties {
    @Test
    @DisplayName("Variables: 4")
    void theNumberOfVariablesIsCorrect() {
      assertEquals(4, problem.numberOfVariables());
    }

    @Test
    @DisplayName("Objectives: 4")
    void theNumberOfObjectivesIsCorrect() {
      assertEquals(4, problem.numberOfObjectives());
    }

    @Test
    @DisplayName("Constraints: 0")
    void theNumberOfConstraintsIsCorrect() {
      assertEquals(0, problem.numberOfConstraints());
    }

    @Test
    @DisplayName("Name: Vaidyanathan2004")
    void theNameIsCorrect() {
      assertEquals("Vaidyanathan2004", problem.name());
    }
  }

  @Nested
  @DisplayName("Its bounds are:")
  class Bounds {
    @ParameterizedTest
    @DisplayName("Lower bounds: ")
    @CsvSource({
        "0, 0.0",
        "1, 0.0",
        "2, 0.0",
        "3, 0.0"
    })
    void checkLowerBounds(int boundIndex, double lowerBound) {
      assertEquals(lowerBound, problem.variableBounds().get(boundIndex).getLowerBound());
    }

    @ParameterizedTest
    @DisplayName("Upper bounds: ")
    @CsvSource({
        "0, 1.0",
        "1, 1.0",
        "2, 1.0",
        "3, 1.0"
    })
    void checkUpperBounds(int boundIndex, double upperBound) {
      assertEquals(upperBound, problem.variableBounds().get(boundIndex).getUpperBound());
    }
  }
}