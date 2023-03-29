function device_outage_parameter!(
    container::PSI.OptimizationContainer,
    T::Type{OutageCommitmentConstraint},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.ThermalGen, W <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    time_steps = PSI.get_time_steps(container)
    device_names = [PSY.get_name(d) for d in devices]
    varon = PSI.get_variable(container, PSI.OnVariable(), V)
    varstop = PSI.get_variable(container, PSI.StopVariable(), V)
    varstart = PSI.get_variable(container, PSI.StartVariable(), V)
    initial_conditions_outage =
        PSI.get_initial_condition(container, InitialOutageStatus(), V)

    con_on = PSI.add_constraints_container!(
        container,
        T(),
        V,
        device_names,
        time_steps,
    )
    con_stop = PSI.add_constraints_container!(
        container,
        T(),
        V,
        device_names,
        time_steps,
        meta = "stop",
    )
    con_start = PSI.add_constraints_container!(
        container,
        T(),
        V,
        device_names,
        time_steps,
        meta = "start",
    )
    param = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), V)
    multiplier =
        PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), V)

    for ic in initial_conditions_outage
        name = PSI.get_component_name(ic)
        con_on[name, 1] = JuMP.@constraint(
            container.JuMPmodel,
            varon[name, 1] <= param[name, 1]
        )
        con_start[name, 1] = JuMP.@constraint(
            container.JuMPmodel,
            varstart[name, 1] <= param[name, 1]
        )
        con_stop[name, 1] = JuMP.@constraint(
            container.JuMPmodel, 
            varstop[name, 1] >= PSI.get_value(ic) - param[name, 1] - (1.0 - varon[name, 1])
        )
        for t in time_steps[2:end]
            con_on[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varon[name, t] <= param[name, t]
            )
            con_start[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstart[name, t] <= param[name, t]
            )
            con_stop[name, t] = JuMP.@constraint(
                container.JuMPmodel, 
                varstop[name, t] >=  param[name, t - 1] - param[name, t] - (1.0 - varon[name, t])
            )

        end
    end
    return
end

function device_outage_ub_parameter!(
    container::PSI.OptimizationContainer,
    T::Type{OutageUpperBoundConstraint},
    ::Type{S},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    ::Type{<:PM.AbstractPowerModel},
) where {
    V <: Union{
        PSY.HydroGen,
        PSY.HydroPumpedStorage,
        PSY.RenewableGen,
        PSY.Storage,
        PSY.ThermalGen,
    },
    S <: PSI.VariableType,
    W <: PSI.AbstractDeviceFormulation,
}
    time_steps = PSI.get_time_steps(container)
    device_names = [PSY.get_name(d) for d in devices]
    varp = PSI.get_variable(container, S(), V)
    constraint = PSI.add_constraints_container!(
        container,
        T(),
        V,
        device_names,
        time_steps,
    )
    param = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), V)
    multiplier =
        PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), V)

    for d in devices, t in time_steps
        name = PSY.get_name(d)
        limits = PSY.get_active_power_limits(d)
        constraint[name, t] = JuMP.@constraint(
            container.JuMPmodel,
            varp[name, t] <= param[name, t] * limits.max
        )
    end
    return
end
