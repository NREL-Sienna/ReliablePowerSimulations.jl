function device_linear_rateofchange_outages!(
    container::PSI.OptimizationContainer,
    T::Type{OutageRampConstraint},
    ::Type{S},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {
    S <: Union{PowerAboveMinimumVariable, ActivePowerVariable},
    V <: PSY.ThermalGen, 
    W <: PSI.AbstractThermalDispatchFormulation
}
    parameters = PSI.built_for_recurrent_solves(container)
    time_steps = PSI.get_time_steps(container)
    variable = PSI.get_variable(container, PSI.ActivePowerVariable(), V)

    ramp_devices = PSI._get_ramp_constraint_devices(container, devices)
    minutes_per_period = PSI._get_minutes_per_period(container)
    IC = PSI._get_initial_condition_type(T, V, W)
    initial_conditions_power = PSI.get_initial_condition(container, IC(), V)

    expr_dn = PSI.get_expression(container, PSI.ActivePowerRangeExpressionLB(), V)
    expr_up = PSI.get_expression(container, PSI.ActivePowerRangeExpressionUB(), V)

    set_name = [PSY.get_name(r) for r in ramp_devices]
    con_up = PSI.add_constraints_container!(container, T(), V, set_name, time_steps, meta="up")
    con_down =
        PSI.add_constraints_container!(container, T(), V, set_name, time_steps, meta="dn")
    outage_parameter = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), V)
    multiplier = PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), V)

    for ic in initial_conditions_power
        name = get_component_name(ic)
        # This is to filter out devices that dont need a ramping constraint
        name ∉ set_name && continue
        ramp_limits = PSY.get_ramp_limits(get_component(ic))
        limits = PSY.get_active_power_limits(get_component(ic))
        ic_power = get_value(ic)
        @debug "add rate_of_change_constraint" name ic_power
        @assert (parameters && isa(ic_power, JuMP.VariableRef)) || !parameters

        con_up[name, 1] = JuMP.@constraint(
            container.JuMPmodel,
            expr_up[name, 1] - ic_power <= ramp_limits.up * minutes_per_period 
            + (limits.min - outage_parameter[name, 1] * limits.min) + limits.max
        )
        con_down[name, 1] = JuMP.@constraint(
            container.JuMPmodel,
            ic_power - expr_dn[name, 1] >= ramp_limits.down * minutes_per_period 
            + (limits.max - outage_parameter[name, 1] * limits.max) + limits.max
        )

        for t in time_steps[2:end]
            con_up[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                expr_up[name, t] - ic_power <= ramp_limits.up * minutes_per_period 
                + (limits.min - outage_parameter[name, t] * limits.min)
            )
            con_down[name, 1] = JuMP.@constraint(
                container.JuMPmodel,
                ic_power - expr_dn[name, 1] >= ramp_limits.down * minutes_per_period 
                + (limits.max - outage_parameter[name, t] * limits.max)
            )
        end
    end
    return
end
