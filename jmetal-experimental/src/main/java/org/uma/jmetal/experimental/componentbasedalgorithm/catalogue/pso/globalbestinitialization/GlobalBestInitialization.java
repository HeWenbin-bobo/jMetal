package org.uma.jmetal.experimental.componentbasedalgorithm.catalogue.pso.globalbestinitialization;

import org.uma.jmetal.solution.doublesolution.DoubleSolution;
import org.uma.jmetal.util.archive.BoundedArchive;

import java.util.List;

public interface GlobalBestInitialization {
  BoundedArchive<DoubleSolution> initialize(List<DoubleSolution> swarm, BoundedArchive<DoubleSolution> globalBest) ;
}
