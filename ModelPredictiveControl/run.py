import model_util as mu
import solver_util as su
import output
import debug


__author__ = 'Erik Wannerberg'


class SimulationManager:
    """
    Class to control the simulation.
    :type _model_obj: model_util.BaseModel
    :type _solver_obj: solver_util.SolverUtil
    :type _output_list: list[output.OutputInterface]
    """
    def __init__(self, model_obj, solver_obj, output_list=list()):
        assert isinstance(model_obj, mu.BaseModel)

        self._model_obj = model_obj  # type: model_util.BaseModel

        assert isinstance(solver_obj, su.SolverUtil)
        self._solver_obj = solver_obj

        if not isinstance(output_list, list):
            assert isinstance(output_list, output.OutputInterface)
            self._output_list = [output_list]
        else:
            assert all(isinstance(o, output.OutputInterface) for o in output_list)
            self._output_list = output_list

        self._error_printer = None

        self.shift_length = 1

    def setup(self):
        """
        Call all the necessary functions in the right order on the participating objects.
        :return:
        :rtype:
        """
        import math
        self._model_obj._create_params()
        self._model_obj._create_variables()
        self._model_obj._create_expressions()
        self._model_obj._create_diffeq_constraints()
        self._model_obj._create_integrals()
        self._model_obj._discretize()
        self._model_obj._create_objectives()
        self._model_obj._presolve()

        self._solver_obj._set_solver(solver_type=self._model_obj._parameters["solver"],
                           solver_executable=self._model_obj._parameters["solver_executable"])
        self.shift_length = math.floor(self._model_obj.parameters["delta_x"]
                                       / self._model_obj.parameters["episode_length"]
                                       * self._model_obj.parameters["episode_pts"])

    def set_error_printer(self, error_printer):
        """
        Set an outputter to print when something goes wrong
        :param error_printer: Outputter at error
        :type error_printer: output.Outputinterface
        """
        assert isinstance(error_printer, output.OutputInterface)

        self._error_printer = error_printer

    def run(self):
        """
        Run the main simulation loop until completion.
        """
        status = self._solver_obj._solve()
        if not status:
            if self._error_printer is not None:
                self._error_printer.print_output()
            debug.CheckInstanceFeasibility(self._model_obj.model, 0.001)

        for output_obj in self._output_list:
            if isinstance(output_obj, output.SavingOutputUtil):
                output_obj.set_index_var(self._model_obj.model.x_local)

                output_obj.add_saved_vars(self._model_obj.model.v,
                                          self._model_obj.model.braking_force,
                                          self._model_obj.model.slope,
                                          self._model_obj.model.height,
                                          self._model_obj.model.inst_cost,
                                          self._model_obj.model.motor_force,)

            output_obj.update()

        while ((self._model_obj.model.global_x_0.value
                + self._model_obj.parameters["episode_length"]
                + self._model_obj.parameters["delta_x"])
               < self._model_obj.parameters["end_dist"]):

            self._model_obj.shift_all(test_model.model.x_local, self.shift_length)
            self._model_obj.model.global_x_0 += sorted(self._model_obj.model.x_local)[self.shift_length]
            self._model_obj._presolve()

            status = self._solver_obj._solve()

            if not status:
                if self._error_printer is not None:
                    self._error_printer.print_output()
                debug.CheckInstanceFeasibility(self._model_obj.model, 0.001)

            for output_obj in self._output_list:
                output_obj.update()

        for output_obj in self._output_list:
            if isinstance(output_obj, output.SavingOutputUtil):
                output_obj.print_output(self._model_obj.parameters["output_file"])
            else:
                output_obj.print_output()


if __name__ == "__main__":
    import sys

    filename = "example/test.json"
    if len(sys.argv) > 1:
        filename = sys.argv[1]

    test_model = mu.VehicleModel(filename)

    solver = su.SolverUtil(test_model)
    outputter = output.SavingOutputUtil(test_model)
    sout = output.SimpleOutputUtil(test_model)

    runner = SimulationManager(model_obj=test_model,
                               solver_obj=solver,
                               output_list=[outputter, sout])
    runner.setup()
    runner.set_error_printer(sout)
    solver._solver.options["max_iter"] = 5000
    solver._solver.options["bound_relax_factor"] = 0.0001  # is actual constraint satisfaction limit
    solver._solver.options["dual_inf_tol"] = 0.1
    solver._solver.options["constr_viol_tol"] = 0.0001
    runner.run()
    solver.print_error_summary()

    if test_model.parameters["plot_after_finish"]:
        outputter.plot_result_graph()
        sout.plot_result_graph()
