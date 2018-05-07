import pyomo
import pyomo.environ as env
import pyomo.dae as dae
import pyomo.core.base
import abc

from params import Params


class BaseModel:
    """
    Base class for building up a pyomo model with some supporting utilities.
    """

    def __init__(self, _model=None):
        if _model is None:
            _model = env.ConcreteModel()

        self._model = _model  # type: pyomo.core.base.Model

    @property
    def model(self):
        """
        Model property getter.
        :return: pyomo model object
        :rtype: pyomo.core.Model
        """
        return self._model

    @abc.abstractmethod
    def _create_variables(self):
        """
        Create the variables.
        """
        pass

    @abc.abstractmethod
    def _create_expressions(self):
        pass

    @abc.abstractmethod
    def _create_diffeq_constraints(self):
        pass

    @abc.abstractmethod
    def _create_integrals(self):
        pass

    @abc.abstractmethod
    def _create_objectives(self):
        pass

    @abc.abstractmethod
    def _discretize(self):
        pass

    @staticmethod
    def _create_const_rule(value):
        def _const_rule(_model, _index):
            return value
        return _const_rule

    @staticmethod
    def _create_lin_int_rule(scale):
        def _lin_interp(_model, _index):
            return scale * _index

        return _lin_interp

    @staticmethod
    def _create_dep_rule(variable_name, functor):
        assert isinstance(variable_name, str)

        def _apply_fun(_model, _index):
            return functor(getattr(_model, variable_name)[_index])

        return _apply_fun

    @staticmethod
    def _fix_with_bounds(variable, value=None, epsilon=0.001):
        if value is None:
            value = env.value(variable)

        variable.setub(value + epsilon)
        variable.setlb(value - epsilon)

    @staticmethod
    def _create_soft_constraint_fn(variable, punish_variable, bounds):
        """
        Helper function to create soft constraint
        :param variable: pyomo variable
        :type variable: pyomo.base.core.Var
        :param punish_variable: variable to take the punish
        :type punish_variable: pyomo.base.core.Var
        :param bounds: tuple with bounds (upper,lower)
        :type bounds: tuple
        :return: a function to generate the constraint
        :rtype: function
        """
        if bounds[0] is None:
            return lambda _mod, _index: variable[_index] - punish_variable[_index] <= bounds[1]
        elif bounds[1] is None:
            return lambda _mod, _index: bounds[0] <= variable[_index] - punish_variable[_index]
        else:
            def _soft_constr_rule(_mod, _index):
                return bounds[0] <= (variable[_index] - punish_variable[_index]) <= bounds[1]

            return _soft_constr_rule


class VehicleModel(BaseModel):

    def __init__(self, parameters, _model=None):
        super().__init__(_model=_model)

        if isinstance(parameters, Params):
            self._parameters = parameters
        elif isinstance(parameters, str):
            self._parameters = Params(parameters)
        else:
            raise TypeError("Unknown type of parameters for model object")

        self.x_local_array = None

    @property
    def parameters(self):
        return self._parameters

    def _create_variables(self):

        _model = self._model
        _parameters = self._parameters
        assert isinstance(_model, pyomo.core.base.Model)

        _model.x_local = dae.ContinuousSet(domain=env.Reals,
                                           bounds=(_parameters["episode_dist_init"], _parameters["episode_dist_end"]),
                                           initialize=(_parameters["episode_dist_init"], _parameters["episode_dist_end"]))

        _model.v                = env.Var(_model.x_local, domain=env.Reals, bounds=_parameters["v_bounds"],
                                          initialize=BaseModel._create_const_rule(_parameters["v_init"]))

        _model.dv_dx            = dae.DerivativeVar(_model.v, wrt=_model.x_local, domain=env.Reals,
                                                    initialize=BaseModel._create_const_rule(0))

        _model.d2v_dx2          = dae.DerivativeVar(_model.v, wrt=(_model.x_local, _model.x_local), domain=env.Reals,
                                                    initialize=BaseModel._create_const_rule(0))

        _model.Temp             = env.Var(_model.x_local, domain=env.Reals, bounds=_parameters["Temp_bounds"],
                                          initialize=BaseModel._create_const_rule(_parameters["Temp_air"]))

        steady_motor_force      = (_model.air_fric_coef * _model.v_target ** 2
                                   + _model.roll_fric_coef * _model.car_mass * _model.grav_const)
        _model.motor_force      = env.Var(_model.x_local, domain=env.Reals, bounds=_parameters["motor_force_bounds"],
                                          initialize=BaseModel._create_const_rule(steady_motor_force))
        _model.dm_dx            = dae.DerivativeVar(_model.motor_force, wrt=_model.x_local,
                                                    # bounds=_parameters["motor_force_deriv_bounds"],
                                                    domain=env.Reals,
                                                    initialize=BaseModel._create_const_rule(0))
        _model.d2m_dx2          = dae.DerivativeVar(_model.motor_force, wrt=(_model.x_local, _model.x_local),
                                                    # bounds=_parameters["motor_force_deriv_bounds"],
                                                    domain=env.Reals,
                                                    initialize=BaseModel._create_const_rule(0))

        _model.braking_force    = env.Var(_model.x_local, domain=env.Reals, bounds=_parameters["braking_force_bounds"],
                                          initialize=BaseModel._create_const_rule(0))
        _model.db_dx            = dae.DerivativeVar(_model.braking_force, wrt=_model.x_local,
                                                    # bounds=_parameters["motor_force_deriv_bounds"], domain=env.Reals,
                                                    initialize=BaseModel._create_const_rule(0))
        _model.d2b_dx2          = dae.DerivativeVar(_model.braking_force, wrt=(_model.x_local, _model.x_local),
                                                    # bounds=_parameters["motor_force_deriv_bounds"], domain=env.Reals,
                                                    initialize=BaseModel._create_const_rule(0))

        _model.height           = env.Var(_model.x_local, domain=env.Reals, initialize=BaseModel._create_const_rule(0))

        _model.dh_dx            = dae.DerivativeVar(_model.height, wrt=_model.x_local, domain=env.Reals,
                                                    initialize=BaseModel._create_const_rule(0))

    def _create_params(self):
        _model = self._model
        _parameters = self._parameters

        _model.car_mass         = env.Param(default=_parameters["car_mass"])
        _model.grav_const       = env.Param(default=_parameters["grav_const"])
        _model.roll_fric_coef   = env.Param(default=_parameters["roll_fric_coef"])
        _model.air_fric_coef    = env.Param(default=_parameters["air_fric_coef"])
        _model.air_cool_coef    = env.Param(default=_parameters["air_cool_coef"])
        _model.cool_exponent    = env.Param(default=_parameters["cool_exponent"])
        _model.Temp_air         = env.Param(default=_parameters["Temp_air"])
        _model.motor_heat_eff   = env.Param(default=_parameters["motor_heat_eff"])
        _model.motor_heat_capac = env.Param(default=_parameters["motor_heat_capac"])
        _model.v_target         = env.Param(default=_parameters["v_target"])
        _model.v_soft_min       = env.Param(default=_parameters["v_soft_min"])
        _model.v_soft_max       = env.Param(default=_parameters["v_soft_max"])
        _model.motor_cost_coef  = env.Param(default=_parameters["motor_cost_coef"])
        _model.vel_cost_coef    = env.Param(default=_parameters["vel_cost_coef"])
        _model.acc_cost_coef    = env.Param(default=_parameters["acc_cost_coef"])
        _model.motor_smooth_coef= env.Param(default=_parameters["motor_smooth_coef"])

        # todo: consider making this a property of VehicleModel
        _model.global_x_0       = env.Param(default=_parameters["init_dist"], mutable=True)

    def _create_expressions(self):
        _model = self._model
        _parameters = self._parameters

        def _to_slope(_mod, _pos):
            return (_mod.dh_dx[_pos]
                    / env.sqrt(1+_mod.dh_dx[_pos]**2))

        _model.v_inv = env.Expression(_model.x_local, rule=lambda _mod, _pos: 1/_mod.v[_pos])

        _model.slope = env.Expression(_model.x_local,
                                      rule=_to_slope)

        _model.gravity = env.Expression(_model.x_local,
                                        rule=lambda _mod, _pos: - (_mod.slope[_pos]
                                                                   * _mod.car_mass
                                                                   * _mod.grav_const))

        _model.drag_force = env.Expression(_model.x_local, rule=(
            lambda _mod, _pos: - _mod.air_fric_coef * (_mod.v[_pos] ** 2)))

        _model.rolling_friction = env.Expression(_model.x_local,
                                                 rule=lambda _mod, _pos: - (
                                                     _mod.roll_fric_coef
                                                     * _mod.car_mass
                                                     * _mod.grav_const
                                                     * (1 - abs(_mod.slope[_pos]))))

        _model.motor_heat = env.Expression(_model.x_local,
                                           rule=lambda _mod, _pos: abs(
                                               _mod.motor_force[_pos]
                                               * _mod.v[_pos]
                                               * (1-_mod.motor_heat_eff)))

        _model.air_cool = env.Expression(_model.x_local,
                                         rule=lambda _mod, _pos: - (
                                             _mod.air_cool_coef
                                             * _mod.v[_pos]
                                             * (_mod.Temp[_pos] - _mod.Temp_air)))

        _model.rad_cool = env.Expression(_model.x_local,
                                         rule=lambda _mod, _pos: - (
                                             _mod.cool_exponent
                                             * (_mod.Temp[_pos] - _mod.Temp_air)))

        _model.steady_force = env.Expression(_model.x_local,
                                             rule=lambda _mod, _pos: (
                                                 - _mod.drag_force[_pos]
                                                 - _mod.rolling_friction[_pos]
                                                 - _mod.gravity[_pos]))

    def _create_diffeq_constraints(self):
        _model = self._model
        _parameters = self._parameters

        def _force_constraint(_mod, _pos):
            """
            Newton's second law, wrt to position.
            :param _mod: pyomo model
            :type _mod: pyomo.core.Model
            :param _pos: position index
            :type _pos: float
            :return: pyomo constraint at position _pos
            :rtype: pyomo.core.base.Expression
            """
            return (_mod.car_mass * _mod.dv_dx[_pos] * _mod.v[_pos]
                    == (_mod.motor_force[_pos]
                        + _mod.braking_force[_pos]
                        + _mod.gravity[_pos]
                        + _mod.drag_force[_pos]
                        + _mod.rolling_friction[_pos]) if _pos <
                                                          _parameters["episode_dist_end"]
                    else env.Constraint.Skip)

        _model.force_constraint = env.Constraint(_model.x_local, rule=_force_constraint)

        def _temp_evolution_constraint(_mod, _pos):
            """
            Heat energy exchange in motor.
            :param _mod: pyomo model
            :type _mod: pyomo.core.Model
            :param _pos: position index
            :type _pos: float
            :return: pyomo constraint at position _pos
            :rtype: pyomo.core.base.Expression
            """
            # TODO: implement
            return env.Constraint.Skip

        # _model.Temp_evolution_constraint = env.Constraint(_model.x_local, rule=_temp_evolution_constraint)

    def _create_integrals(self):
        _model = self._model
        _parameters = self._parameters

        def _force_expr(_mod, _pos):
            # to offset for having optimum at the target velocity
            steady_motor_force_coef = 2 * _mod.air_fric_coef * _mod.v_target ** 3  # THERE SHOULD BE A FACTOR 2 IN THERE BUT IF IT'S THERE IT MESSES IT UP ARGH. AAAAARGH i found it.

            # add a constant to show cost departure from equilibrium rather than abs
            cost_balancer = (_mod.air_fric_coef * _mod.v_target ** 2
                             - _mod.rolling_friction[_pos]
                             - _mod.gravity[_pos]
                             + steady_motor_force_coef / _mod.v_target)
            return (_mod.motor_force[_pos] + steady_motor_force_coef * _mod.v_inv[_pos]
                    - cost_balancer
                    if _pos < _parameters["episode_dist_end"] else 0)

        # for soft constraint on velocity
        _model.v_softlim = env.Var(_model.x_local, domain=env.Reals,
                                   initialize=BaseModel._create_const_rule(0))
        _model.v_softconstraint = env.Constraint(_model.x_local,
                                                 rule=BaseModel._create_soft_constraint_fn(
                                                     _model.v,
                                                     _model.v_softlim,
                                                     (_model.v_soft_min,
                                                      _model.v_soft_max)))

        def _motor_smooth_expr(_mod, _pos):
            return (_mod.d2m_dx2[_pos] ** 2
                    + _mod.d2b_dx2[_pos] ** 2
                    if _pos < _parameters["episode_dist_end"] else 0)

        def _vel_expr(_mod, _pos):
            return (abs(_mod.v_softlim[_pos])**2 * _mod.v_inv[_pos]
                    if _pos < _parameters["episode_dist_end"] else 0)

        def _v_2ndtimederiv_expr(_mod, _pos):
            return (_mod.v[_pos]*(_mod.dv_dx[_pos]**2 +
                                  _mod.v[_pos] * _mod.dv_dx[_pos].derivative(_mod.x_local)
                                  )**2
                    if _pos < _parameters["episode_dist_end"] else 0)

        _model.energy_integral      = dae.Integral(_model.x_local, wrt=_model.x_local,
                                                   rule=_force_expr)
        _model.velocity_integral    = dae.Integral(_model.x_local, wrt=_model.x_local,
                                                   rule=_vel_expr)
        _model.acc_smooth_integral  = dae.Integral(_model.x_local, wrt=_model.x_local,
                                                   rule=_v_2ndtimederiv_expr)
        _model.motor_smooth_integral= dae.Integral(_model.x_local, wrt=_model.x_local,
                                                   rule=_motor_smooth_expr)

        _model.inst_cost = env.Expression(_model.x_local,
                                          rule=lambda _mod, _pos: (
                                              _mod.motor_cost_coef * _force_expr(_mod, _pos)
                                              + _mod.vel_cost_coef * _vel_expr(_mod, _pos)
                                              + _mod.acc_cost_coef * _v_2ndtimederiv_expr(_mod, _pos)
                                              + _mod.motor_smooth_coef * _motor_smooth_expr(_mod, _pos)
                                              # to compensate for the cost compensation
                                              - _mod.gravity[_pos]
                                              - _mod.rolling_friction[_pos]))

    def _discretize(self, _transform_type="dae.finite_difference",
                    _scheme="forward", **kwargs):

        _model = self._model
        _parameters = self._parameters

        discretizer = env.TransformationFactory(_transform_type)
        discretizer.apply_to(_model, nfe=_parameters["episode_pts"],
                             wrt=_model.x_local, scheme=_scheme, **kwargs)

        for expr in _model.component_objects(env.Expression):
            expr.reconstruct()

    def _create_objectives(self):
        _model = self._model
        _parameters = self._parameters

        def _int_rule(_mod):
            return (
                _mod.motor_cost_coef * _mod.energy_integral
                + _mod.vel_cost_coef * _mod.velocity_integral
                + _mod.acc_cost_coef * _mod.acc_smooth_integral
                + _mod.motor_smooth_coef * _mod.motor_smooth_integral
            )

        _model.objective_integral = env.Objective(rule=_int_rule, sense=env.minimize)

    def _fix_initial(self):
        _model = self._model

        x_sorted = sorted(_model.x_local)

        if env.value(_model.global_x_0) == self.parameters["init_dist"]:
            # initialising variables more properly to avoid first infeasible
            _model.motor_force[x_sorted[0]] = env.value(_model.steady_force[x_sorted[0]])
            _model.motor_force[x_sorted[1]] = env.value(_model.steady_force[x_sorted[1]])
            _model.motor_force[x_sorted[2]] = env.value(_model.steady_force[x_sorted[2]])
            _model.motor_force[x_sorted[3]] = env.value(_model.steady_force[x_sorted[3]])
            _model.dm_dx[x_sorted[0]] = _model.dm_dx.get_derivative_expression()(x_sorted[0])()
            _model.dm_dx[x_sorted[1]] = _model.dm_dx.get_derivative_expression()(x_sorted[1])()
            _model.dm_dx[x_sorted[2]] = _model.dm_dx.get_derivative_expression()(x_sorted[2])()

        BaseModel._fix_with_bounds(_model.v[x_sorted[0]])
        BaseModel._fix_with_bounds(_model.v[x_sorted[1]])
        BaseModel._fix_with_bounds(_model.v[x_sorted[2]])
        BaseModel._fix_with_bounds(_model.motor_force[x_sorted[0]])
        BaseModel._fix_with_bounds(_model.dm_dx[x_sorted[0]])
        BaseModel._fix_with_bounds(_model.d2m_dx2[x_sorted[0]])
        BaseModel._fix_with_bounds(_model.braking_force[x_sorted[0]])
        BaseModel._fix_with_bounds(_model.Temp[x_sorted[0]])

    def _fix_final(self):
        _model = self._model

        x_sorted = sorted(_model.x_local)
        BaseModel._fix_with_bounds(_model.v[x_sorted[-1]],
                                   value=env.value(_model.v_target), epsilon=0.001)

        # _model.v[x_sorted[-1]].fix(_model.v_target.value)

    def _presolve(self):
        import numpy as np
        _model = self._model
        _parameters = self._parameters

        if self.x_local_array is None:
            self.x_local_array = np.array(sorted(_model.x_local))

        def load_height(x_0, height_var, deriv_var, index_var):
            smoothened_heights = _parameters.get_gaussmooth_height(
                x_0 + self.x_local_array,
                _parameters["height_smoothing_length"])

            for i, _index in enumerate(sorted(index_var)):
                height_var[_index] = smoothened_heights[i]
                # height_var[_index] = float(_parameters.get_spline_height(x_0 + _index))
                BaseModel._fix_with_bounds(height_var[_index], epsilon=0.0001)
                # height_var[_index].fix(_parameters.get_li_height(x_0 + _index))

            VehicleModel.fix(var=deriv_var, index_var=index_var,
                             length_of_fix=0, unfix_length=len(index_var))

            VehicleModel._set_to_derivative_val(deriv_var=deriv_var, index_var=index_var)

            VehicleModel.fix(var=deriv_var, index_var=index_var,
                             length_of_fix=len(index_var))

        load_height(env.value(_model.global_x_0), _model.height,
                    _model.dh_dx, _model.x_local)

        self._fix_initial()
        self._fix_final()
        self.presolved = True
        # TODO: implement... more?

    @staticmethod
    def shift(var, index_var, length_of_shift):
        sorted_ind = sorted(index_var)
        index_length = len(sorted_ind)

        for i in range(index_length - length_of_shift):
            var[sorted_ind[i]] = env.value(var[sorted_ind[i+length_of_shift]])
            var[sorted_ind[i]].stale = False

        for i in range(index_length - length_of_shift, index_length):
            # -2 since sometimes the last index has weird stuff going on
            var[sorted_ind[i]] = env.value(var[sorted_ind[index_length - length_of_shift - 2]])
            var[sorted_ind[i]].stale = False

    @staticmethod
    def fix(var, index_var, length_of_fix, unfix_length=0):
        sorted_ind = sorted(index_var)
        index_length = len(sorted_ind)

        for i in range(length_of_fix):
            var[sorted_ind[i]].fix()

        for i in range(length_of_fix, unfix_length):
            var[sorted_ind[i]].unfix()

    def shift_all(self, index_var, length_of_shift):
        for var in self.model.component_objects(env.Var):
            if not isinstance(var, dae.DerivativeVar):
                # print("Shifting {}".format(var.name))
                VehicleModel.shift(var, index_var, length_of_shift)
        for var in self.model.component_objects(env.Var):
            if isinstance(var, dae.DerivativeVar):
                # print("Setting deriv var {}".format(var.name))
                VehicleModel._set_to_derivative_val(var, index_var)

    @staticmethod
    def _set_to_derivative_val(deriv_var, index_var):
        """
        Set value of variable to the evaluated derivative
        :param deriv_var: derivative variable
        :type deriv_var: pyomo.dae.DerivativeVar
        :param index_var: set variable
        :type index_var: pyomo.core.base.ContinuousSet
        """
        for i in index_var:
            try:
                deriv_var[i] = deriv_var.get_derivative_expression()(i)()
            except IndexError as e:
                continue
                # print("Index [{}] out of bounds for {}".format(i, deriv_var.name))

