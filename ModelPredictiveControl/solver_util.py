import pyomo.environ as env
import model_util as mu
from pyutilib.common import ApplicationError
from pyomo.opt import TerminationCondition, SolverStatus

__author__ = 'Erik Wannerberg'


class SolverUtil:

    def __init__(self, model_utils):
        assert isinstance(model_utils, mu.BaseModel)

        self._model_obj = model_utils
        self._solver = None
        self._failure_counter = 0
        self._solver_errs = []

    def _set_solver(self, solver_type='ipopt', solver_executable=None, options=None):

        if solver_executable is None:
            opt = env.SolverFactory(solver_type)
        else:
            opt = env.SolverFactory(solver_type, executable=solver_executable, )

        if options is None:
            # use old settings if there are some
            if self._solver is not None:
                opt.options = self._solver.options

            # these options worked well so far
            opt.options["max_iter"] = 1500
            opt.options["bound_relax_factor"] = 0.001  # is actual constraint satisfaction limit
            opt.options["dual_inf_tol"] = 0.1
            opt.options["constr_viol_tol"] = 0.001
        else:
            opt.options = options

        self._solver = opt

    def _solve(self):
        _solver = self._solver
        _model = self._model_obj._model #TODO: make this access proper
        results = None
        try:
            print("Starting solve")
            results = _solver.solve(_model, tee=False, report_timing=True)
            print("Solve finished")
            if(results.solver.termination_condition == TerminationCondition.infeasible
               or results.solver.termination_condition == TerminationCondition.maxIterations
               or results.solver.status != SolverStatus.ok):
                success = False
                self._failure_counter += 1
                self._solver_errs += [results.solver.termination_condition]
            else:
                success = True
            self._results = results
        except (ApplicationError, ValueError) as err:
            print("ERROR! ", err)
            success = False
            self._failure_counter += 1
            if results is None:
                self._solver_errs += [err]
            else:
                self._solver_errs += [err, results.solver.status,
                                      results.solver.termination_condition]

        return success

    def print_error_summary(self):
        print("Solver failures: {}".format(self._failure_counter))
        for msg in self._solver_errs:
            print(msg)
