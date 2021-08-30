"""
This function creates the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, ThermalStandardUCOutages},
    ::Type{S},
) where {T <: PSY.ThermalGen, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReactivePowerVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.OnVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.StartVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.StopVariable,
        devices,
        ThermalStandardUCOutages(),
    )

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, ThermalStandardUCOutages())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.commitment_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.ramp_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.time_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

"""
This function creates the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, ThermalStandardUCOutages},
    ::Type{S},
) where {T <: PSY.ThermalGen, S <: PM.AbstractActivePowerModel}
    devices = get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.OnVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.StartVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.StopVariable,
        devices,
        ThermalStandardUCOutages(),
    )

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, ThermalStandardUCOutages())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.commitment_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.ramp_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.time_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function construct_device!(
    optimization_container::OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, ThermalDispatchOutages},
    ::Type{S},
) where {
    T <: PSY.ThermalGen,
    S <: PM.AbstractPowerModel,
}
    devices = get_available_components(T, sys)

    if !validate_available_devices(T, devices)
        return
    end

    # Variables
    add_variables!(optimization_container, ActivePowerVariable, devices, ThermalDispatchOutages())
    add_variables!(optimization_container, ReactivePowerVariable, devices, ThermalDispatchOutages())

    # Initial Conditions

    # Constraints
    add_constraints!(
        optimization_container,
        RangeConstraint,
        ActivePowerVariable,
        devices,
        model,
        S,
        get_feedforward(model),
    )
    add_constraints!(
        optimization_container,
        RangeConstraint,
        ReactivePowerVariable,
        devices,
        model,
        S,
        get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    feedforward!(optimization_container, devices, model, get_feedforward(model))

    # Cost Function
    cost_function!(optimization_container, devices, model, S, get_feedforward(model))

    return
end

function construct_device!(
    optimization_container::OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, ThermalDispatchOutages},
    ::Type{S},
) where {
    T <: PSY.ThermalGen,
    S <: PM.AbstractActivePowerModel,
}
    devices = get_available_components(T, sys)

    if !validate_available_devices(T, devices)
        return
    end

    # Variables
    add_variables!(optimization_container, ActivePowerVariable, devices, ThermalDispatchOutages())

    # Initial Conditions

    # Constraints
    add_constraints!(
        optimization_container,
        RangeConstraint,
        ActivePowerVariable,
        devices,
        model,
        S,
        get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    feedforward!(optimization_container, devices, model, get_feedforward(model))

    # Cost Function
    cost_function!(optimization_container, devices, model, S, get_feedforward(model))

    return
end
