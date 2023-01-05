function device_linear_rateofchange_outages!(
    container::PSI.OptimizationContainer,
    T::Type{OutageRampConstraint},
    rate_data::Vector{DeviceOutageRampConstraintInfo},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
) where {V <: PSY.ThermalGen, W <: PSI.AbstractThermalDispatchFormulation}
    parameters = PSI.built_for_recurrent_solves(container)
    time_steps = PSI.get_time_steps(container)
    variable = PSI.get_variable(container, PSI.ActivePowerVariable(), V)

    ramp_devices = PSI._get_ramp_constraint_devices(container, devices)
    minutes_per_period = PSI._get_minutes_per_period(container)
    # IC = PSI._get_initial_condition_type(T, V, W)
    # initial_conditions_power = PSI.get_initial_condition(container, IC(), V)

    set_name = [PSY.get_name(r) for r in ramp_devices]
    con_up = PSI.add_constraints_container!(container, T(), V, set_name, time_steps, meta="up")
    con_down =
        PSI.add_constraints_container!(container, T(), V, set_name, time_steps, meta="dn")
    outage_parameter = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), V)
    multiplier = PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), V)

    for r in rate_data
        name = get_component_name(r)
        ic_power = PSI.get_value(get_ic_power(r))
        ic_outage = PSI.get_value(get_ic_outage(r))
        @debug "add rate_of_change_constraint" name ic_power
        @assert (parameters && isa(ic_power, PJ.ParameterRef)) || !parameters
        expression_ub = JuMP.AffExpr(0.0, variable[name, 1] => 1.0)
        for val in r.additional_terms_ub
            JuMP.add_to_expression!(
                expression_ub,
                PSI.get_variable(container, val)[name, 1],
            )
        end
        con_up[name, 1] = JuMP.@constraint(
            container.JuMPmodel,
            expression_ub - ic_power <=
            r.ramp_limits.up + (r.limits.min - ic_outage * r.limits.min) + r.limits.max
        )
        expression_lb = JuMP.AffExpr(0.0, variable[name, 1] => 1.0)
        for val in r.additional_terms_lb
            JuMP.add_to_expression!(
                expression_lb,
                PSI.get_variable(container, val)[name, 1],
                -1.0,
            )
        end
        con_down[name, 1] = JuMP.@constraint(
            container.JuMPmodel,
            ic_power - expression_lb <=
            r.ramp_limits.down +
            (r.limits.max - outage_parameter[name, 1] * r.limits.max) +
            r.limits.max
        )
    end

    for t in time_steps[2:end], r in rate_data
        name = get_component_name(r)
        expression_ub = JuMP.AffExpr(0.0, variable[name, t] => 1.0)
        for val in r.additional_terms_ub
            JuMP.add_to_expression!(
                expression_ub,
                PSI.get_variable(container, val)[name, t],
            )
        end
        con_up[name, t] = JuMP.@constraint(
            container.JuMPmodel,
            expression_ub - variable[name, t - 1] <=
            r.ramp_limits.up +
            (r.limits.min - outage_parameter[name, t - 1] * r.limits.min)
        )
        expression_lb = JuMP.AffExpr(0.0, variable[name, t] => 1.0)
        for val in r.additional_terms_lb
            JuMP.add_to_expression!(
                expression_lb,
                PSI.get_variable(container, val)[name, t],
                -1.0,
            )
        end
        con_down[name, t] = JuMP.@constraint(
            container.JuMPmodel,
            variable[name, t - 1] - expression_lb <=
            r.ramp_limits.down + (r.limits.max - outage_parameter[name, t] * r.limits.max)
        )
    end

    return
end

function device_mixedinteger_rateofchange_outages!(
    container::PSI.OptimizationContainer,
    T::Type{OutageRampConstraint},
    rate_data::Vector{DeviceOutageRampConstraintInfo},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
) where {V <: PSY.ThermalGen, W <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    parameters = PSI.built_for_recurrent_solves(container)
    time_steps = PSI.get_time_steps(container)
    variable = PSI.get_variable(container, PSI.ActivePowerVariable(), V)
    varstart = PSI.get_variable(container, PSI.StartVariable(), V)
    varstop = PSI.get_variable(container, PSI.StopVariable(), V)

    ramp_devices = PSI._get_ramp_constraint_devices(container, devices)
    minutes_per_period = PSI._get_minutes_per_period(container)
    # IC = PSI._get_initial_condition_type(T, V, W)
    # initial_conditions_power = PSI.get_initial_condition(container, IC(), V)

    set_name = [PSY.get_name(r) for r in ramp_devices]
    con_up = PSI.add_constraints_container!(container, T(), V, set_name, time_steps, meta="up")
    con_down =
        PSI.add_constraints_container!(container, T(), V, set_name, time_steps, meta="dn")
    outage_parameter = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), V)
    multiplier = PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), V)

    for r in rate_data
        name = r.component_name
        ic_power = PSI.get_value(r.ic_power)
        ic_outage = PSI.get_value(r.ic_outage)
        @debug "add rate_of_change_constraint" name ic_power
        @assert (parameters && isa(ic_power, PJ.ParameterRef)) || !parameters
        expression_ub = JuMP.AffExpr(0.0, variable[name, 1] => 1.0)
        for val in r.additional_terms_ub
            JuMP.add_to_expression!(
                expression_ub,
                PSI.get_variable(container, val)[name, 1],
            )
        end
        con_up[name, 1] = JuMP.@constraint(
            container.JuMPmodel,
            expression_ub - (ic_power) <=
            r.ramp_limits.up + r.limits.max * varstart[name, 1]
        )
        expression_lb = JuMP.AffExpr(0.0, variable[name, 1] => 1.0)
        for val in r.additional_terms_lb
            JuMP.add_to_expression!(
                expression_lb,
                PSI.get_variable(container, val)[name, 1],
                -1.0,
            )
        end
        con_down[name, 1] = JuMP.@constraint(
            container.JuMPmodel,
            (ic_power) - expression_lb <=
            r.ramp_limits.down + r.limits.max * varstop[name, 1]
        )
    end

    for t in time_steps[2:end], r in rate_data
        name = get_component_name(r)
        expression_ub = JuMP.AffExpr(0.0, variable[name, t] => 1.0)
        for val in r.additional_terms_ub
            JuMP.add_to_expression!(
                expression_ub,
                PSI.get_variable(container, val)[name, t],
            )
        end
        con_up[name, t] = JuMP.@constraint(
            container.JuMPmodel,
            expression_ub - variable[name, t - 1] <=
            r.ramp_limits.up + r.limits.max * varstart[name, t]
        )
        expression_lb = JuMP.AffExpr(0.0, variable[name, t] => 1.0)
        for val in r.additional_terms_lb
            JuMP.add_to_expression!(
                expression_lb,
                PSI.get_variable(container, val)[name, t],
                -1.0,
            )
        end
        con_down[name, t] = JuMP.@constraint(
            container.JuMPmodel,
            variable[name, t - 1] - expression_lb <=
            r.ramp_limits.down + r.limits.max * varstop[name, t]
        )
    end

    return
end