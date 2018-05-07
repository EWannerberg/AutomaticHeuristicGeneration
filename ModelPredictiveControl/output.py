__author__ = 'erik'
import model_util as mu
import abc


class OutputInterface:

    def __init__(self, model_utils):
            assert isinstance(model_utils, mu.BaseModel)

            self._model_obj = model_utils

    @abc.abstractmethod
    def print_output(self):
        pass

    @abc.abstractmethod
    def update(self):
        """
        Get notified and gather new data from model object.
        """
        pass


class SimpleOutputUtil(OutputInterface):
    """
    Just prints the model to stdout.
    """
    def __init__(self, model_utils):
        super().__init__(model_utils)

    def update(self):
        import pyomo.environ as env

        _model = self._model_obj.model

        print("Current distance {:.2f}/{:.0f}".format(env.value(_model.global_x_0), self._model_obj.parameters["end_dist"]))
        print('Energy cost int.: {:.4g}'.format(env.value((_model.motor_cost_coef * _model.energy_integral))),
              ', Vel. cost int.: {:.4g}'.format(env.value((_model.vel_cost_coef * _model.velocity_integral))),
              ', Acc. cost int.: {:.4g}'.format(env.value((_model.acc_cost_coef * _model.acc_smooth_integral))),
              ', Force smooth int.: {:.4g}'.format(env.value(_model.motor_smooth_coef * _model.motor_smooth_integral)))
        pass

    def print_output(self):
        """
        Just prints the model to stdout.
        """
        import pyomo.environ as env
        _model = self._model_obj.model
        _model.display()
        print('Energy cost int.: {:.4g}'.format(env.value((_model.motor_cost_coef * _model.energy_integral))),
              ', Vel. cost int.: {:.4g}'.format(env.value((_model.vel_cost_coef * _model.velocity_integral))),
              ', Acc. cost int.: {:.4g}'.format(env.value((_model.acc_cost_coef * _model.acc_smooth_integral))),
              ', Force smooth int.: {:.4g}'.format(env.value(_model.motor_smooth_coef * _model.motor_smooth_integral)))

    def plot_result_graph(self):
        import matplotlib.pyplot as plt
        import pyomo.environ as env

        _model = self._model_obj._model
        _model_obj = self._model_obj

        assert isinstance(_model_obj, mu.VehicleModel)
        _parameters = _model_obj._parameters

        def euler_integrate(indices, deriv_var_fn, wrt_var_fn):
            indices_sorted = sorted(indices)
            integral = [0]
            for j in range(1, len(indices_sorted)):
                integral.append(integral[j-1] +
                                env.value((deriv_var_fn(indices_sorted[j])
                                           + deriv_var_fn(indices_sorted[j])) / 2
                                          * (wrt_var_fn(indices_sorted[j])
                                             - wrt_var_fn(indices_sorted[j-1]))))
            return integral

        plt.figure()

        index_var = sorted(_model.x_local)
        x_axis = []
        history_x_axis = []
        if _parameters._params.get("plot_on_x_axis", True):
            x_label_text = "Distance"
            x_axis = index_var
        else:
            x_label_text = "Time"
            x_axis = euler_integrate(index_var,
                                     lambda x: 1/_model.v[x], lambda x: _model.x_local[x])

        for var in _model.component_objects(ctype=env.Var):
            var_values = [env.value(var[ind]) for ind in index_var]
            line, = plt.plot(x_axis, var_values)
            line.set_label(var.name)

        plt.plot([x_axis[0], x_axis[-1]], [_parameters["v_max"], _parameters["v_max"]],
                 color='g', linestyle='dashed')
        plt.plot([x_axis[0], x_axis[-1]], [_parameters["v_min"], _parameters["v_min"]],
                 color='g', linestyle='dashed')

        plt.xlabel(x_label_text)
        plt.ylabel("Variable value")
        plt.legend(loc=3)

        plt.show()

    def plot_result_graph(self):
        import matplotlib.pyplot as plt
        import pyomo.environ as env

        _model = self._model_obj._model
        _model_obj = self._model_obj

        assert isinstance(_model_obj, mu.VehicleModel)
        _parameters = _model_obj._parameters

        def euler_integrate(indices, deriv_var_fn, wrt_var_fn):
            indices_sorted = sorted(indices)
            integral = [0]
            for j in range(1, len(indices_sorted)):
                integral.append(integral[j-1] +
                                env.value((deriv_var_fn(indices_sorted[j])
                                           + deriv_var_fn(indices_sorted[j])) / 2
                                          * (wrt_var_fn(indices_sorted[j])
                                             - wrt_var_fn(indices_sorted[j-1]))))
            return integral

        plt.figure()

        index_var = sorted(_model.x_local)
        x_axis = []
        history_x_axis = []
        if _parameters._params.get("plot_on_x_axis", True):
            x_label_text = "Distance"
            x_axis = index_var
        else:
            x_label_text = "Time"
            x_axis = euler_integrate(index_var,
                                     lambda x: 1/_model.v[x], lambda x: _model.x_local[x])

        for var in _model.component_objects(ctype=env.Var):
            var_values = []
            for ind in index_var:
                try:
                    var_values += [env.value(var[ind])]
                except (IndexError, ValueError):
                    break
            line, = plt.plot(x_axis[0:len(var_values)], var_values)
            line.set_label(var.name)

        plt.plot([x_axis[0], x_axis[-1]], [_parameters["v_max"], _parameters["v_max"]],
                 color='g', linestyle='dashed')
        plt.plot([x_axis[0], x_axis[-1]], [_parameters["v_min"], _parameters["v_min"]],
                 color='g', linestyle='dashed')

        plt.xlabel(x_label_text)
        plt.ylabel("Variable value")
        plt.legend(loc=3)

        plt.show()


class SavingOutputUtil(OutputInterface):

    def __init__(self, model_utils):
        assert isinstance(model_utils, mu.VehicleModel)
        super().__init__(model_utils)
        self._index_var = None
        self._vars_to_save = []
        self._saved_data = dict()
        self._update_print_counter = 0

    def set_index_var(self, index_var):

        if index_var.name not in self._saved_data.keys():
            self._saved_data[index_var.name] = []
        self._index_var = index_var

    def add_saved_vars(self, *variables):
        self._vars_to_save += variables

        for var in variables:
            if var.name not in self._saved_data.keys():
                self._saved_data[var.name] = []

    def update(self):
        """
        Save the current values in the model.
        """
        import pyomo.environ as env
        _model = self._model_obj.model  # type: pyomo.core.base.Model
        _index_var = self._index_var
        sorted_ind = sorted(_index_var)

        _saved_index = self._saved_data[_index_var.name]

        # iterate from end
        current_pos_index = len(_saved_index) - (next((x for x, val in
                                                       enumerate(reversed(_saved_index))
                                                       if val < _model.global_x_0.value),
                                                      0))

        _saved_index = _saved_index[0:current_pos_index] + [_model.global_x_0.value + x
                                                            for x in sorted_ind]
        self._saved_data[_index_var.name] = _saved_index

        # overwrite from current pos all the new values currently in the var
        for var in self._vars_to_save:
            self._saved_data[var.name] = (
                self._saved_data[var.name][0:current_pos_index]
                + [env.value(var[ind]) for ind in sorted_ind]
            )

        self._update_print_counter += 1
        if self._update_print_counter > self._model_obj.parameters["output_intermediate_after"]:
            print("After {} value savings, saving intermediate output file...".format(self._update_print_counter))
            self._update_print_counter = 0
            self.print_output(intermediate=True)

    def print_output(self, filename=None, intermediate=False):
        """
        Print output saved through update()
        """
        import get_inputs as gi

        if filename is None:
            try:
                filename = self._model_obj.parameters["output_file"]
            except (KeyError, AttributeError):
                filename = "out.csv"

        gi.write_output_csv(filename, intermediate=intermediate, **self._saved_data)

    def plot_result_graph(self, index_key="x_local", saved_results=None):
        import matplotlib.pyplot as plt
        import pyomo.environ as env

        if saved_results is None:
            saved_results = self._saved_data

        plt.figure()

        def euler_integrate(indices, deriv_var_fn, wrt_var_fn):
            indices_sorted = sorted(indices)
            integral = [0]
            for j in range(1, len(indices_sorted)):
                integral.append(integral[j-1] +
                                env.value((deriv_var_fn(indices_sorted[j])
                                           + deriv_var_fn(indices_sorted[j])) / 2
                                          * (wrt_var_fn(indices_sorted[j])
                                             - wrt_var_fn(indices_sorted[j-1]))))
            return integral

        index_var = saved_results[index_key]
        x_axis = []
        plot_on_x_axis = True
        if plot_on_x_axis:
            x_label_text = "Distance"
            x_axis = index_var
        else:
            x_label_text = "Time"
            x_axis = euler_integrate(index_var,
                                     lambda x: 1/saved_results["v"][x],
                                     lambda x: saved_results["x_local"][x])

        for key, var in saved_results.items():
            if key == index_key:
                continue
            line, = plt.plot(x_axis, var)
            line.set_label(key)

        plt.xlabel(x_label_text)
        plt.ylabel("Variable value")
        plt.legend(loc=3)

        plt.show()
