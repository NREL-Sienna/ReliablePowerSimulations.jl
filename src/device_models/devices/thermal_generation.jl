############## AuxiliaryOnVariable, ThermalGen ####################
PSI.get_variable_binary(::AuxiliaryOnVariable, ::Type{<:PSY.ThermalGen}, _) = true
PSI.get_variable_lower_bound(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) = 0.0
PSI.get_variable_upper_bound(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) = 1.0

PSI.get_variable_binary(::AuxiliaryStartVariable, ::Type{<:PSY.ThßermalGen}, _) = true
PSI.get_variable_lower_bound(::AuxiliaryStartVariable, d::PSY.ThermalGen, _) = 0.0
PSI.get_variable_upper_bound(::AuxiliaryStartVariable, d::PSY.ThermalGen, _) = 1.0

PSI.get_variable_binary(::AuxiliaryStopVariable, ::Type{<:PSY.ThermalGen}, _) = true
PSI.get_variable_lower_bound(::AuxiliaryStopVariable, d::PSY.ThermalGen, _) = 0.0
PSI.get_variable_upper_bound(::AuxiliaryStopVariable, d::PSY.ThermalGen, _) = 1.0

PSI.initial_condition_default(
    ::InitialOutageStatus,
    d::PSY.ThermalGen,
    ::PSI.AbstractThermalFormulation,
) = PSY.get_status(d) ? 1 : 0

PSI.initial_condition_variable(
    ::InitialOutageStatus,
    d::PSY.ThermalGen,
    ::PSI.AbstractThermalFormulation,
) = PSI.OnVariable()

PSI.get_variable_lower_bound(
    ::PSI.ActivePowerVariable,
    d::PSY.ThermalGen,
    ::Union{ThermalNoMinOutages, ThermalNoMinRampLimitedOutages},
) = 0.0

PSI._get_initial_condition_type(
    ::Type{OutageRampConstraint},
    ::Type{<:PSY.ThermalGen},
    ::Type{<:AbstractThermalOutageCommitmentFormulation},
) = PSI.DevicePower

PSI.get_multiplier_value(
    ::PSI.TimeSeriesParameter,
    d::PSY.ThermalGen,
    ::AbstractThermalOutageDispatchFormulation,
) = 1.0

PSI.get_multiplier_value(
    ::PSI.TimeSeriesParameter,
    d::PSY.ThermalGen,
    ::AbstractThermalOutageCommitmentFormulation,
) = 1.0

function PSI.get_default_time_series_names(
    ::Type{<:PSY.ThermalGen},
    ::Type{
        <:Union{
            AbstractThermalOutageDispatchFormulation,
            AbstractThermalOutageCommitmentFormulation,
        },
    },
)
    return Dict{Type{<:PSI.TimeSeriesParameter}, String}(
        OutageTimeSeriesParameter => "outage",
    )
end

######## CONSTRAINTS ############

"""
Min and max active power limits of generators for thermal dispatch no minimum formulations
"""
function PSI.get_min_max_limits(
    device,
    ::Type{PSI.ActivePowerVariableLimitsConstraint},
    ::Type{S},
) where {S <: Union{ThermalNoMinOutages, ThermalNoMinRampLimitedOutages}}
    return (min = 0.0, max = PSY.get_active_power_limits(device).max)
end

### Outage Range Constraints

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{<:PSI.PowerVariableLimitsConstraint},
    U::Type{<:Union{PSI.VariableType, PSI.ExpressionType}},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.ThermalGen, W <: AbstractThermalOutageDispatchFormulation}
    if !has_semicontinuous_outage_feedforward(model, U)
        PSI.add_range_constraints!(container, T, U, devices, model, X)
    end
    return
end

### Outage Ramp Constraints

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    ::Type{OutageRampConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::PSI.DeviceModel{U, V},
    W::Type{<:PM.AbstractPowerModel},
) where {U <: PSY.ThermalGen, V <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    add_semicontinuous_ramp_constraints_outages!(
        container,
        OutageRampConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        W,
    )
    return
end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageRampConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::PSI.DeviceModel{U, V},
    W::Type{<:PM.AbstractPowerModel},
) where {U <: PSY.ThermalGen, V <: PSI.AbstractThermalDispatchFormulation}
    device_linear_rateofchange_outages!(
        container,
        T,
        PSI.ActivePowerVariable,
        devices,
        model,
        W,
    )
    return
end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageRampConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::PSI.DeviceModel{U, V},
    W::Type{<:PM.AbstractPowerModel},
) where {U <: PSY.ThermalGen, V <: ThermalNoMinRampLimitedOutages}
    parameters = PSI.built_for_recurrent_solves(container)
    time_steps = PSI.get_time_steps(container)

    ramp_devices = PSI._get_ramp_constraint_devices(container, devices)
    minutes_per_period = PSI._get_minutes_per_period(container)
    IC = PSI._get_initial_condition_type(T, V, W)
    initial_conditions_power = PSI.get_initial_condition(container, IC(), V)

    expr_dn = PSI.get_expression(container, PSI.ActivePowerRangeExpressionLB(), V)
    expr_up = PSI.get_expression(container, PSI.ActivePowerRangeExpressionUB(), V)

    set_name = [PSY.get_name(r) for r in ramp_devices]
    con_up =
        PSI.add_constraints_container!(container, T(), V, set_name, time_steps, meta = "up")
    con_down =
        PSI.add_constraints_container!(container, T(), V, set_name, time_steps, meta = "dn")

    outage_parameter = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), T)
    multiplier =
        PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), T)

    for ic in initial_conditions_power
        name = PSI.get_component_name(ic)
        # This is to filter out devices that dont need a ramping constraint
        name ∉ set_name && continue

        ramp_limits = PSY.get_ramp_limits(PSI.get_component(ic))
        limits = PSY.get_active_power_limits(PSI.get_component(ic))
        ic_power = PSI.get_value(ic)
        @debug "add rate_of_change_constraint" name ic_power
        @assert (parameters && isa(ic_power, JuMP.VariableRef)) || !parameters
        con_up[name, 1] = JuMP.@constraint(
            container.JuMPmodel,
            expr_up[name, 1] - ic_power <= ramp_limits.up * minutes_per_period
        )
        con_down[name, 1] = JuMP.@constraint(
            container.JuMPmodel,
            ic_power - expr_dn[name, 1] >=
            ramp_limits.down * minutes_per_period +
            (limits.max - outage_parameter[name, 1] * limits.max)
        )
        for t in time_steps[2:end]
            con_up[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                expr_up[name, t] - variable[name, t - 1] <=
                ramp_limits.up * minutes_per_period
            )
            con_down[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                variable[name, t - 1] - expr_dn[name, t] >=
                ramp_limits.down * minutes_per_period +
                (limits.max - outage_parameter[name, t] * limits.max)
            )
        end
    end
    return
end

## Outage Commitment Constraints

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageCommitmentConstraint},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.ThermalGen, W <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    device_outage_parameter!(container, T, devices, model, X)

    return
end

## Outage Bound Constraints

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageUpperBoundConstraint},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.ThermalGen, W <: AbstractThermalOutageDispatchFormulation}
    device_outage_ub_parameter!(container, T, PSI.ActivePowerVariable, devices, model, X)
    return
end

### Time Constraints

function _get_data_for_tdc(
    initial_conditions_on::Vector{T},
    initial_conditions_off::Vector{U},
    initial_conditions_outage::Vector{V},
    resolution::Dates.TimePeriod,
) where {T <: PSI.InitialCondition, U <: PSI.InitialCondition, V <: PSI.InitialCondition}
    steps_per_hour = 60 / Dates.value(Dates.Minute(resolution))
    fraction_of_hour = 1 / steps_per_hour
    lenght_devices_on = length(initial_conditions_on)
    lenght_devices_off = length(initial_conditions_off)
    lenght_devices_outage = length(initial_conditions_outage)

    IS.@assert_op lenght_devices_off == lenght_devices_on
    IS.@assert_op lenght_devices_off == lenght_devices_outage
    time_params = Vector{PSI.UpDown}(undef, lenght_devices_on)
    ini_conds = Matrix{PSI.InitialCondition}(undef, lenght_devices_on, 3)
    idx = 0
    for (ix, ic) in enumerate(initial_conditions_on)
        g = PSI.get_component(ic)
        IS.@assert_op g == PSI.get_component(initial_conditions_off[ix])
        time_limits = PSY.get_time_limits(g)
        name = PSY.get_name(g)
        if time_limits !== nothing
            if (time_limits.up <= fraction_of_hour) &&
               (time_limits.down <= fraction_of_hour)
                @debug "Generator $(name) has a nonbinding time limits. Constraints Skipped"
                continue
            else
                idx += 1
            end
            ini_conds[idx, 1] = ic
            ini_conds[idx, 2] = initial_conditions_off[ix]
            ini_conds[idx, 3] = initial_conditions_outage[ix]
            up_val = round(time_limits.up * steps_per_hour, RoundUp)
            down_val = round(time_limits.down * steps_per_hour, RoundUp)
            time_params[idx] = time_params[idx] = (up = up_val, down = down_val)
        end
    end
    if idx < lenght_devices_on
        ini_conds = ini_conds[1:idx, :]
        deleteat!(time_params, (idx + 1):lenght_devices_on)
    end
    return ini_conds, time_params
end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    ::Type{OutageTimeConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    ::PSI.DeviceModel{U, V},
    ::Type{<:PM.AbstractPowerModel},
) where {U <: PSY.ThermalGen, V <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    resolution = PSI.get_resolution(container)
    initial_conditions_on =
        PSI.get_initial_condition(container, PSI.InitialTimeDurationOn(), U)
    initial_conditions_off =
        PSI.get_initial_condition(container, PSI.InitialTimeDurationOff(), U)
    initial_conditions_outage =
        PSI.get_initial_condition(container, InitialOutageStatus(), U)
    ini_conds, time_params = _get_data_for_tdc(
        initial_conditions_on,
        initial_conditions_off,
        initial_conditions_outage,
        resolution,
    )

    parameters = PSI.built_for_recurrent_solves(container)
    if !(isempty(ini_conds))
        if parameters
            device_duration_parameters_outage!(
                container,
                time_params,
                ini_conds,
                OutageTimeConstraint(),
                U,
            )
        else
            device_duration_look_ahead_outage!(
                container,
                time_params,
                ini_conds,
                OutageTimeConstraint(),
                U,
            )
        end
    end
    return
end

######## Initial Condition ############

function PSI.initial_conditions!(
    container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::D,
) where {T <: PSY.ThermalGen, D <: AbstractThermalOutageDispatchFormulation}
    PSI.add_initial_condition!(container, devices, formulation, PSI.DevicePower())
    PSI.add_initial_condition!(container, devices, formulation, InitialOutageStatus())
    return
end

function PSI.initial_conditions!(
    container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::D,
) where {T <: PSY.ThermalGen, D <: Union{ThermalStandardUCOutages}}
    PSI.add_initial_condition!(container, devices, formulation, PSI.DeviceStatus())
    PSI.add_initial_condition!(container, devices, formulation, PSI.DevicePower())
    PSI.add_initial_condition!(container, devices, formulation, PSI.InitialTimeDurationOn())
    PSI.add_initial_condition!(
        container,
        devices,
        formulation,
        PSI.InitialTimeDurationOff(),
    )
    PSI.add_initial_condition!(container, devices, formulation, InitialOutageStatus())
    return
end

function PSI.initial_conditions!(
    container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::D,
) where {T <: PSY.ThermalGen, D <: Union{ThermalBasicUCOutages}}
    PSI.add_initial_condition!(container, devices, formulation, PSI.DeviceStatus())
    PSI.add_initial_condition!(container, devices, formulation, PSI.DevicePower())
    PSI.add_initial_condition!(container, devices, formulation, InitialOutageStatus())
    return
end

######## Auxiliary Variable Calculation ############

function calculate_aux_variable_value!(
    container::OptimizationContainer,
    ::AuxVarKey{DeviceStatus, T},
    system::PSY.System,
) where {T <: PSY.ThermalGen}
    devices = get_available_components(T, system)
    time_steps = get_time_steps(container)

    if has_container_key(container, OnVariable, T)
        on_variable_results = get_variable(container, OnVariable(), T)
    elseif has_container_key(container, OnStatusParameter, T)
        on_variable_results = get_parameter_array(container, OnStatusParameter(), T)
    else
        on_variable_results = get_parameter_array(container, OutageTimeSeriesParameter(), T)
    end

    outage_param = get_parameter_array(container, OutageTimeSeriesParameter(), T)
    aux_variable_container = get_aux_variable(container, DeviceStatus(), T)
    for d in devices, t in time_steps
        name = PSY.get_name(d)
        aux_variable_container[name, t] =
            jump_value(on_variable_results[name, t]) * jump_value(outage_param[name, t])
    end
    return
end

######## Objective Function ############

function PSI._add_pwl_term!(
    container::PSI.OptimizationContainer,
    component::T,
    data::Vector{NTuple{2, Float64}},
    ::U,
    ::V,
) where {
    T <: PSY.Component,
    U <: PSI.VariableType,
    V <: AbstractThermalOutageDispatchFormulation,
}
    multiplier = PSI.objective_function_multiplier(U(), V())
    resolution = PSI.get_resolution(container)
    dt = Dates.value(Dates.Second(resolution)) / PSI.SECONDS_IN_HOUR
    base_power = PSI.get_base_power(container)
    # Re-scale breakpoints by Basepower
    name = PSY.get_name(component)

    is_power_data_compact = PSI._check_pwl_compact_data(component, data, base_power)

    if !PSI.uses_compact_power(component, V()) && is_power_data_compact
        error(
            "The data provided is not compatible with formulation $V. Use a formulation compatible with Compact Cost Functions",
        )
        # data = _convert_to_full_variable_cost(data, component)
    elseif PSI.uses_compact_power(component, V()) && !is_power_data_compact
        data = PSI._convert_to_compact_variable_cost(data)
    else
        @debug PSI.uses_compact_power(component, V()) name T V
        @debug is_power_data_compact name T V
    end

    slopes = PSY.get_slopes(data)
    # First element of the return is the average cost at P_min.
    # Shouldn't be passed for convexity check
    is_convex = PSI._slope_convexity_check(slopes[2:end])
    time_steps = PSI.get_time_steps(container)
    pwl_cost_expressions = Vector{JuMP.AffExpr}(undef, time_steps[end])
    break_points = map(x -> last(x), data) ./ base_power
    sos_val = PSI._get_sos_value(container, V, component)
    for t in time_steps
        PSI._add_pwl_variables!(container, T, name, t, data)
        _add_pwl_constraint_outages!(container, component, U(), break_points, sos_val, t)
        if !is_convex
            PSI._add_pwl_sos_constraint!(
                container,
                component,
                U(),
                break_points,
                sos_val,
                t,
            )
        end
        pwl_cost =
            PSI._get_pwl_cost_expression(container, component, t, data, multiplier * dt)
        pwl_cost_expressions[t] = pwl_cost
    end
    return pwl_cost_expressions
end

function _add_pwl_constraint_outages!(
    container::PSI.OptimizationContainer,
    component::T,
    ::U,
    break_points::Vector{Float64},
    sos_status::PSI.SOSStatusVariable,
    period::Int,
) where {T <: PSY.Component, U <: PSI.VariableType}
    variables = PSI.get_variable(container, U(), T)
    const_container = PSI.lazy_container_addition!(
        container,
        PSI.PieceWiseLinearCostConstraint(),
        T,
        axes(variables)...,
    )
    len_cost_data = length(break_points)
    jump_model = PSI.get_jump_model(container)
    pwl_vars = PSI.get_variable(container, PSI.PieceWiseLinearCostVariable(), T)
    name = PSY.get_name(component)
    const_container[name, period] = JuMP.@constraint(
        jump_model,
        variables[name, period] ==
        sum(pwl_vars[name, ix, period] * break_points[ix] for ix in 1:len_cost_data)
    )

    if sos_status == PSI.SOSStatusVariable.NO_VARIABLE
        bin = 1.0
        @debug "Using Piecewise Linear cost function but no variable/parameter ref for ON status is passed. Default status will be set to online (1.0)" _group =
            PSI.LOG_GROUP_COST_FUNCTIONS

    elseif sos_status == PSI.SOSStatusVariable.PARAMETER
        bin = PSI.get_parameter(container, PSI.OnStatusParameter(), T).parameter_array[
            name,
            period,
        ]
        @debug "Using Piecewise Linear cost function with parameter OnStatusParameter, $T" _group =
            PSI.LOG_GROUP_COST_FUNCTIONS
    elseif sos_status == PSI.SOSStatusVariable.VARIABLE
        bin = PSI.get_variable(container, PSI.OnVariable(), T)[name, period]
        @debug "Using Piecewise Linear cost function with variable OnVariable $T" _group =
            PSI.LOG_GROUP_COST_FUNCTIONS
    else
        @assert false
    end

    JuMP.@constraint(
        jump_model,
        sum(pwl_vars[name, i, period] for i in 1:len_cost_data) <= bin
    )
    return
end
