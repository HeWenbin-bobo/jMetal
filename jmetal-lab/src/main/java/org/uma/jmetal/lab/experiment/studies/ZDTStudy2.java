// To change the save format of results files, can modify "runAlgorithm"
// in "\jMetal\jmetal-lab\src\main\java\org\\uma\jmetal\lab\experiment\component\impl\ExecuteAlgorithms.java"
package org.uma.jmetal.lab.experiment.studies;

import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.algorithm.multiobjective.moead.AbstractMOEAD;
import org.uma.jmetal.algorithm.multiobjective.moead.MOEADBuilder;
import org.uma.jmetal.algorithm.multiobjective.nsgaii.NSGAIIBuilder;
import org.uma.jmetal.algorithm.multiobjective.smpso.SMPSOBuilder;
import org.uma.jmetal.algorithm.multiobjective.abyss.ABYSSBuilder;
import org.uma.jmetal.algorithm.multiobjective.agemoea.AGEMOEABuilder;
import org.uma.jmetal.algorithm.multiobjective.agemoeaii.AGEMOEAIIBuilder;
import org.uma.jmetal.algorithm.multiobjective.cdg.CDGBuilder;
import org.uma.jmetal.algorithm.multiobjective.smsemoa.SMSEMOABuilder;
import org.uma.jmetal.algorithm.multiobjective.espea.ESPEABuilder;
import org.uma.jmetal.algorithm.multiobjective.espea.util.EnergyArchive.ReplacementStrategy;
import org.uma.jmetal.algorithm.multiobjective.fame.FAME;
import org.uma.jmetal.algorithm.multiobjective.gwasfga.GWASFGA;
import org.uma.jmetal.algorithm.multiobjective.microfame.MicroFAME;
import org.uma.jmetal.algorithm.multiobjective.microfame.util.HVTournamentSelection;
import org.uma.jmetal.algorithm.multiobjective.mombi.MOMBI2;
import org.uma.jmetal.lab.experiment.Experiment;
import org.uma.jmetal.lab.experiment.ExperimentBuilder;
import org.uma.jmetal.lab.experiment.component.impl.*;
import org.uma.jmetal.lab.experiment.util.ExperimentAlgorithm;
import org.uma.jmetal.lab.experiment.util.ExperimentProblem;
import org.uma.jmetal.lab.visualization.StudyVisualizer;
import org.uma.jmetal.operator.crossover.CrossoverOperator;
import org.uma.jmetal.operator.crossover.impl.DifferentialEvolutionCrossover;
import org.uma.jmetal.operator.crossover.impl.SBXCrossover;
import org.uma.jmetal.operator.crossover.impl.NullCrossover;
import org.uma.jmetal.operator.mutation.MutationOperator;
import org.uma.jmetal.operator.mutation.impl.PolynomialMutation;
import org.uma.jmetal.operator.mutation.impl.NullMutation;
import org.uma.jmetal.operator.selection.SelectionOperator;
import org.uma.jmetal.operator.selection.impl.RandomSelection;
import org.uma.jmetal.operator.selection.impl.SpatialSpreadDeviationSelection;
import org.uma.jmetal.operator.selection.impl.BinaryTournamentSelection;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.problem.doubleproblem.DoubleProblem;
import org.uma.jmetal.problem.multiobjective.zdt.*;
import org.uma.jmetal.qualityindicator.impl.*;
import org.uma.jmetal.qualityindicator.impl.hypervolume.impl.PISAHypervolume;
import org.uma.jmetal.solution.doublesolution.DoubleSolution;
import org.uma.jmetal.util.archive.impl.CrowdingDistanceArchive;
import org.uma.jmetal.util.errorchecking.JMetalException;
import org.uma.jmetal.util.evaluator.impl.SequentialSolutionListEvaluator;
import org.uma.jmetal.util.legacy.qualityindicator.impl.hypervolume.Hypervolume;
import org.uma.jmetal.util.comparator.RankingAndCrowdingDistanceComparator;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Example of experimental study based on solving the ZDT problems with the algorithms NSGAII,
 * MOEA/D, and SMPSO.
 *
 * This experiment assumes that the reference Pareto front are known and that, given a problem named
 * P, there is a corresponding file called P.csv containing its corresponding Pareto front. If this
 * is not the case, please refer to class {@link DTLZStudy} to see an example of how to explicitly
 * indicate the name of those files.
 *
 * Five quality indicators are used for performance assessment: {@link Epsilon}, {@link Spread},
 * {@link GenerationalDistance}, {@link PISAHypervolume}, and {@link InvertedGenerationalDistancePlus}.
 *
 * The steps to carry out are:
 * 1. Configure the experiment
 * 2. Execute the algorithms
 * 3. Compute que quality indicators
 * 4. Generate Latex tables reporting means and medians, and tables with statistical tests
 * 5. Generate HTML pages with tables, boxplots, and fronts.
 *
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */

public class ZDTStudy2 {
  private static final int INDEPENDENT_RUNS = 5;
  private static final int POPULATION_SIZE = 100;
  private static final int MAX_EVALUATIONS = 25000;
  private static final int MAX_ITERATIONS = 250;

  public static void main(String[] args) throws IOException {
//    if (args.length != 1) {
//      throw new JMetalException("Missing argument: experimentBaseDirectory");
//    }
    String experimentBaseDirectory = "G:\\jMetal";

    List<ExperimentProblem<DoubleSolution>> problemList = List.of(
            new ExperimentProblem<>(new ZDT1()),
            // new ExperimentProblem<>(new ZDT1().setReferenceFront("front.csv"))
            new ExperimentProblem<>(new ZDT2()),
            new ExperimentProblem<>(new ZDT3()),
            new ExperimentProblem<>(new ZDT4()),
            new ExperimentProblem<>(new ZDT5Modified()),
            new ExperimentProblem<>(new ZDT6()));

    List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithmList =
            configureAlgorithmList(problemList);

    Experiment<DoubleSolution, List<DoubleSolution>> experiment =
            new ExperimentBuilder<DoubleSolution, List<DoubleSolution>>("ZDTStudy2")
                    .setAlgorithmList(algorithmList)
                    .setProblemList(problemList)
                    .setReferenceFrontDirectory("resources/referenceFrontsCSV")
                    .setExperimentBaseDirectory(experimentBaseDirectory)
                    .setOutputParetoFrontFileName("FUN")
                    .setOutputParetoSetFileName("VAR")
                    .setOutputTimeFileName("TIME")
                    .setIndicatorList(List.of(
                            new Epsilon(),
                            new Spread(),
                            new GenerationalDistance(),
                            new PISAHypervolume(),
                            new NormalizedHypervolume(),
                            new InvertedGenerationalDistance(),
                            new InvertedGenerationalDistancePlus()))
                    .setIndependentRuns(INDEPENDENT_RUNS)
                    .setNumberOfCores(8)
                    .build();

    new ExecuteAlgorithms<>(experiment).run();
    new ComputeQualityIndicators<>(experiment).run();
    new GenerateLatexTablesWithStatistics(experiment).run();
    new GenerateFriedmanHolmTestTables<>(experiment).run();
    new GenerateWilcoxonTestTablesWithR<>(experiment).run();
    new GenerateBoxplotsWithR<>(experiment).setRows(2).setColumns(3).run();
    new GenerateHtmlPages<>(experiment, StudyVisualizer.TYPE_OF_FRONT_TO_SHOW.MEDIAN).run() ;
  }

  /**
   * The algorithm list is composed of pairs {@link Algorithm} + {@link Problem} which form part of
   * a {@link ExperimentAlgorithm}, which is a decorator for class {@link Algorithm}.
   */
  static List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> configureAlgorithmList(
          List<ExperimentProblem<DoubleSolution>> problemList) {
    List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms = new ArrayList<>();
    for (int run = 0; run < INDEPENDENT_RUNS; run++) {
      for (var experimentProblem : problemList) {
        smpso(algorithms, run, experimentProblem);
        nsgaii(algorithms, run, experimentProblem);
        moead(algorithms, run, experimentProblem);
        abyss(algorithms, run, experimentProblem);
        agemoea(algorithms, run, experimentProblem);
        agemoeaii(algorithms, run, experimentProblem);
        cdg(algorithms, run, experimentProblem);
        smsemoa(algorithms, run, experimentProblem);
        espa(algorithms, run, experimentProblem);
        fame(algorithms, run, experimentProblem);
        gwasfga(algorithms, run, experimentProblem);
        microfame(algorithms, run, experimentProblem);
        mobi2(algorithms, run, experimentProblem);
      }
    }
    return algorithms;
  }

  private static void moead(
      List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms, int run,
      ExperimentProblem<DoubleSolution> experimentProblem) {
    Algorithm<List<DoubleSolution>> algorithm = new MOEADBuilder(experimentProblem.getProblem(), MOEADBuilder.Variant.MOEAD)
            .setCrossover(new DifferentialEvolutionCrossover(1.0, 0.5, DifferentialEvolutionCrossover.DE_VARIANT.RAND_1_BIN))
            .setMutation(new PolynomialMutation(1.0 / experimentProblem.getProblem().numberOfVariables(),
                    20.0))
            .setMaxEvaluations(MAX_EVALUATIONS)
            .setPopulationSize(POPULATION_SIZE)
            .setResultPopulationSize(POPULATION_SIZE)
            .setNeighborhoodSelectionProbability(0.9)
            .setMaximumNumberOfReplacedSolutions(2)
            .setNeighborSize(20)
            .setFunctionType(AbstractMOEAD.FunctionType.TCHE)
            .build();

    algorithms.add(new ExperimentAlgorithm<>(algorithm, experimentProblem, run));
  }

  private static void nsgaii(
      List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms, int run,
      ExperimentProblem<DoubleSolution> experimentProblem) {
    Algorithm<List<DoubleSolution>> algorithm = new NSGAIIBuilder<DoubleSolution>(
            experimentProblem.getProblem(),
            new SBXCrossover(1.0, 20.0),
            new PolynomialMutation(1.0 / experimentProblem.getProblem().numberOfVariables(),
                    20.0),
            POPULATION_SIZE)
            .setMaxEvaluations(MAX_EVALUATIONS)
            .build();
    algorithms.add(new ExperimentAlgorithm<>(algorithm, experimentProblem, run));
  }

  private static void smpso(
      List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms, int run,
      ExperimentProblem<DoubleSolution> experimentProblem) {
    double mutationProbability = 1.0 / experimentProblem.getProblem().numberOfVariables();
    double mutationDistributionIndex = 20.0;
    Algorithm<List<DoubleSolution>> algorithm = new SMPSOBuilder(
            (DoubleProblem) experimentProblem.getProblem(),
            new CrowdingDistanceArchive<DoubleSolution>(POPULATION_SIZE))
            .setMutation(new PolynomialMutation(mutationProbability, mutationDistributionIndex))
            .setMaxIterations(MAX_ITERATIONS)
            .setSwarmSize(POPULATION_SIZE)
            .setSolutionListEvaluator(new SequentialSolutionListEvaluator<>())
            .build();
    algorithms.add(new ExperimentAlgorithm<>(algorithm, experimentProblem, run));
  }

  private static void abyss(
          List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms, int run,
          ExperimentProblem<DoubleSolution> experimentProblem) {
    Algorithm<List<DoubleSolution>> algorithm = new ABYSSBuilder(
            (DoubleProblem) experimentProblem.getProblem(),
            new CrowdingDistanceArchive<DoubleSolution>(POPULATION_SIZE))
            .setMaxEvaluations(MAX_EVALUATIONS)
            .setPopulationSize(POPULATION_SIZE)
            .build();
    algorithms.add(new ExperimentAlgorithm<>(algorithm, experimentProblem, run));
  }

  private static void agemoea(
          List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms, int run,
          ExperimentProblem<DoubleSolution> experimentProblem) {
    double crossoverProbability = 0.9;
    double crossoverDistributionIndex = 30.0;
    var crossover = new SBXCrossover(crossoverProbability, crossoverDistributionIndex);

    double mutationProbability = 1.0 / experimentProblem.getProblem().numberOfVariables();
    double mutationDistributionIndex = 20.0;
    var mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex);

    Algorithm<List<DoubleSolution>> algorithm = new AGEMOEABuilder<>(
            (DoubleProblem) experimentProblem.getProblem())
            .setCrossoverOperator(crossover)
            .setMutationOperator(mutation)
            .setMaxIterations(MAX_ITERATIONS)
            .setPopulationSize(POPULATION_SIZE)
            .setSolutionListEvaluator(new SequentialSolutionListEvaluator<>())
            .build();
    algorithms.add(new ExperimentAlgorithm<>(algorithm, experimentProblem, run));
  }

  private static void agemoeaii(
          List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms, int run,
          ExperimentProblem<DoubleSolution> experimentProblem) {
    double crossoverProbability = 0.9;
    double crossoverDistributionIndex = 30.0;
    var crossover = new SBXCrossover(crossoverProbability, crossoverDistributionIndex);

    double mutationProbability = 1.0 / experimentProblem.getProblem().numberOfVariables();
    double mutationDistributionIndex = 20.0;
    var mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex);

    Algorithm<List<DoubleSolution>> algorithm = new AGEMOEAIIBuilder<>(
            (DoubleProblem) experimentProblem.getProblem())
            .setCrossoverOperator(crossover)
            .setMutationOperator(mutation)
            .setMaxIterations(MAX_ITERATIONS)
            .setPopulationSize(POPULATION_SIZE)
            .setSolutionListEvaluator(new SequentialSolutionListEvaluator<>())
            .build();
    algorithms.add(new ExperimentAlgorithm<>(algorithm, experimentProblem, run));
  }

  private static void cdg(
          List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms, int run,
          ExperimentProblem<DoubleSolution> experimentProblem) {
    double cr = 1.0;
    double f = 0.5;
    var crossover =
            new DifferentialEvolutionCrossover(
                    cr, f, DifferentialEvolutionCrossover.DE_VARIANT.RAND_1_BIN);

    Algorithm<List<DoubleSolution>> algorithm = new CDGBuilder(
            (DoubleProblem) experimentProblem.getProblem())
            .setCrossover(crossover)
            .setMaxEvaluations(MAX_EVALUATIONS)
            .setPopulationSize(POPULATION_SIZE)
            .setResultPopulationSize(POPULATION_SIZE)
            .setNeighborhoodSelectionProbability(0.9)
            .build();
    algorithms.add(new ExperimentAlgorithm<>(algorithm, experimentProblem, run));
  }

  private static void smsemoa(
          List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms, int run,
          ExperimentProblem<DoubleSolution> experimentProblem) {
    double crossoverProbability = 0.9;
    double crossoverDistributionIndex = 20.0;
    CrossoverOperator<DoubleSolution> crossover = new SBXCrossover(crossoverProbability,
            crossoverDistributionIndex);

    double mutationProbability = 1.0 / experimentProblem.getProblem().numberOfVariables();
    double mutationDistributionIndex = 20.0;
    MutationOperator<DoubleSolution> mutation = new PolynomialMutation(mutationProbability,
            mutationDistributionIndex);

    var selection = new RandomSelection<DoubleSolution>();

//    Hypervolume<DoubleSolution> hypervolume;
//    hypervolume = new PISAHypervolume<>();
//    hypervolume.setOffset(100.0);

    Algorithm<List<DoubleSolution>> algorithm = new SMSEMOABuilder<>(
            (DoubleProblem) experimentProblem.getProblem(), crossover, mutation)
            .setSelectionOperator(selection)
            .setMaxEvaluations(MAX_EVALUATIONS)
            .setPopulationSize(POPULATION_SIZE)
            // .setHypervolumeImplementation(hypervolume)
            .build();
    algorithms.add(new ExperimentAlgorithm<>(algorithm, experimentProblem, run));
  }

  private static void espa(
          List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms, int run,
          ExperimentProblem<DoubleSolution> experimentProblem) {
    double crossoverProbability = 0.9;
    double crossoverDistributionIndex = 20.0;
    var crossover = new SBXCrossover(crossoverProbability, crossoverDistributionIndex);

    double mutationProbability = 1.0 / experimentProblem.getProblem().numberOfVariables();
    double mutationDistributionIndex = 20.0;
    var mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex);

    ESPEABuilder<DoubleSolution> builder = new ESPEABuilder<>(
            (DoubleProblem) experimentProblem.getProblem(), crossover, mutation);
    builder.setMaxEvaluations(MAX_EVALUATIONS);
    builder.setPopulationSize(POPULATION_SIZE);
    builder.setReplacementStrategy(ReplacementStrategy.WORST_IN_ARCHIVE);
    Algorithm<List<DoubleSolution>> algorithm = builder.build();
    algorithms.add(new ExperimentAlgorithm<>(algorithm, experimentProblem, run));
  }

  private static void fame(
          List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms, int run,
          ExperimentProblem<DoubleSolution> experimentProblem) {
    SelectionOperator<List<DoubleSolution>, DoubleSolution> selection =
            new SpatialSpreadDeviationSelection<>(5);

    Algorithm<List<DoubleSolution>> algorithm = new FAME<>(
            (DoubleProblem) experimentProblem.getProblem(),
            POPULATION_SIZE,
            POPULATION_SIZE,
            MAX_EVALUATIONS,
            selection,
            new SequentialSolutionListEvaluator<>()
    );
    algorithms.add(new ExperimentAlgorithm<>(algorithm, experimentProblem, run));
  }

  private static void gwasfga(
          List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms, int run,
          ExperimentProblem<DoubleSolution> experimentProblem) {
    double crossoverProbability = 0.9;
    double crossoverDistributionIndex = 20.0;
    CrossoverOperator<DoubleSolution> crossover = new SBXCrossover(crossoverProbability,
            crossoverDistributionIndex);

    double mutationProbability = 1.0 / experimentProblem.getProblem().numberOfVariables();
    double mutationDistributionIndex = 20.0;
    MutationOperator<DoubleSolution> mutation = new PolynomialMutation(mutationProbability,
            mutationDistributionIndex);

    SelectionOperator<List<DoubleSolution>, DoubleSolution> selection = new BinaryTournamentSelection<>(
            new RankingAndCrowdingDistanceComparator<>());

    double epsilon = 0.01;

    Algorithm<List<DoubleSolution>> algorithm = new GWASFGA<>(
            (DoubleProblem) experimentProblem.getProblem(),
            POPULATION_SIZE,
            MAX_ITERATIONS,
            crossover,
            mutation,
            selection,
            new SequentialSolutionListEvaluator<>(),
            epsilon
    );
    algorithms.add(new ExperimentAlgorithm<>(algorithm, experimentProblem, run));
  }

  private static void microfame(
          List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms, int run,
          ExperimentProblem<DoubleSolution> experimentProblem) {
    CrossoverOperator<DoubleSolution> crossover = new NullCrossover<>();
    MutationOperator<DoubleSolution> mutation = new NullMutation<>();
    SelectionOperator<List<DoubleSolution>, DoubleSolution> selection = new HVTournamentSelection(
            5);

    Algorithm<List<DoubleSolution>> algorithm = new MicroFAME<>(
            (DoubleProblem) experimentProblem.getProblem(),
            MAX_EVALUATIONS,
            POPULATION_SIZE,
            crossover,
            mutation,
            selection
    );
    algorithms.add(new ExperimentAlgorithm<>(algorithm, experimentProblem, run));
  }

  private static void mobi2(
          List<ExperimentAlgorithm<DoubleSolution, List<DoubleSolution>>> algorithms, int run,
          ExperimentProblem<DoubleSolution> experimentProblem) {
    double crossoverProbability = 0.9;
    double crossoverDistributionIndex = 20.0;
    CrossoverOperator<DoubleSolution> crossover = new SBXCrossover(crossoverProbability,
            crossoverDistributionIndex);

    double mutationProbability = 1.0 / experimentProblem.getProblem().numberOfVariables();
    double mutationDistributionIndex = 20.0;
    MutationOperator<DoubleSolution> mutation = new PolynomialMutation(mutationProbability,
            mutationDistributionIndex);

    SelectionOperator<List<DoubleSolution>, DoubleSolution> selection = new BinaryTournamentSelection<>(
            new RankingAndCrowdingDistanceComparator<>());

    Algorithm<List<DoubleSolution>> algorithm = new MOMBI2<>(
            (DoubleProblem) experimentProblem.getProblem(),
            MAX_ITERATIONS,
            crossover,
            mutation,
            selection,
            new SequentialSolutionListEvaluator<DoubleSolution>(),
            "resources/weightVectorFiles/mombi2/weight_02D_200.sld"
    );
    algorithms.add(new ExperimentAlgorithm<>(algorithm, experimentProblem, run));
  }

}
