package org.uma.jmetal.lab.experiment.util;

import java.io.File;
import java.util.List;
import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.lab.experiment.Experiment;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;

/**
 * Class defining tasks for the execution of algorithms in parallel.
 *
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
public class ExperimentAlgorithm<S extends Solution<?>, Result extends List<S>> {
  private Algorithm<Result> algorithm;
  private String algorithmTag;
  private String problemTag;
  private int runId;

  /** Constructor */
  public ExperimentAlgorithm(
      Algorithm<Result> algorithm, String algorithmTag, ExperimentProblem<S> problem, int runId) {
    this.algorithm = algorithm;
    this.algorithmTag = algorithmTag;
    this.problemTag = problem.getTag();
    this.runId = runId;
  }

  public ExperimentAlgorithm(Algorithm<Result> algorithm, ExperimentProblem<S> problem, int runId) {

    this(algorithm, algorithm.name(), problem, runId);
  }

  public void runAlgorithm(Experiment<?, ?> experimentData) {
    String outputDirectoryName =
        experimentData.getExperimentBaseDirectory() + "/data/" + algorithmTag + "/" + problemTag;

    File outputDirectory = new File(outputDirectoryName);
    if (!outputDirectory.exists()) {
      boolean result = new File(outputDirectoryName).mkdirs();
      if (result) {
        JMetalLogger.logger.info("Creating " + outputDirectoryName);
      } else {
        JMetalLogger.logger.severe("Creating " + outputDirectoryName + " failed");
      }
    }

    String funFile =
        outputDirectoryName + "/" + experimentData.getOutputParetoFrontFileName() + "." + runId + ".tsv";
    String varFile =
        outputDirectoryName + "/" + experimentData.getOutputParetoSetFileName() + "." + runId + ".tsv";
    String timeFile =
        outputDirectoryName + "/" + experimentData.getOutputTimeFileName() + "." + runId;
    String message = " Running algorithm: "
        + algorithmTag
        + ", problem: "
        + problemTag
        + ", run: "
        + runId
        + ", funFile: "
        + funFile ;
    JMetalLogger.logger.info(message);

    try {
      double stime = (double)System.currentTimeMillis();
      algorithm.run();
      double etime = (double)System.currentTimeMillis();
      Result population = algorithm.result();

      new SolutionListOutput(population, (etime-stime) / 1000)
          .setVarFileOutputContext(new DefaultFileOutputContext(varFile, " "))
          .setFunFileOutputContext(new DefaultFileOutputContext(funFile, " "))
          .setTimeFileOutputContext(new DefaultFileOutputContext(timeFile, " "))
          .print();
//      new SolutionListOutput(population, (etime-stime) / 1000)
//              .setVarFileOutputContext(new DefaultFileOutputContext(varFile, ","))
//              .setFunFileOutputContext(new DefaultFileOutputContext(funFile, ","))
//              .setTimeFileOutputContext(new DefaultFileOutputContext(timeFile, ","))
//              .print();
    } catch (Exception exception) {
      JMetalLogger.logger.warning("Execution failed: " + funFile + " has not been created.");
    }
  }

  public Algorithm<Result> getAlgorithm() {
    return algorithm;
  }

  public String getAlgorithmTag() {
    return algorithmTag;
  }

  public String getProblemTag() {
    return problemTag;
  }

  public int getRunId() {
    return this.runId;
  }
}
