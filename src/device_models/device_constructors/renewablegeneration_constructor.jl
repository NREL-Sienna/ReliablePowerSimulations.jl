function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{R, D},
    ::Type{S},
) where {R <: PSY.RenewableGen, D <: RenewableOutageDispatch, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(R, sys)

    if !PSI.validate_available_devices(R, devices)
        return
    end

    # Variables
    PSI.add_variables!(optimization_container, PSI.ActivePowerVariable, devices, D())
    PSI.add_variables!(optimization_container, PSI.ReactivePowerVariable, devices, D())

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
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.eedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(optimization_container, devices, model, S)

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{R, D},
    ::Type{S},
) where {
    R <: PSY.RenewableGen,
    D <: RenewableOutageDispatch,
    S <: PM.AbstractActivePowerModel,
}
    devices = PSI.get_available_components(R, sys)

    if !PSI.validate_available_devices(R, devices)
        return
    end

    # Variables
    PSI.add_variables!(optimization_container, PSI.ActivePowerVariable, devices, D())

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
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(optimization_container, devices, model, S)

    return
end
