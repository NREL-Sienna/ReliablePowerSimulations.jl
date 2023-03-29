function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{St, D},
    ::Type{S},
) where {St <: PSY.Storage, D <: BookKeepingOutage, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(St, sys)

    PSI.add_variables!(container, PSI.ActivePowerInVariable, devices, D())
    PSI.add_variables!(container, PSI.ActivePowerOutVariable, devices, D())
    PSI.add_variables!(container, PSI.ReactivePowerVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergyVariable, devices, D())
    if PSI.get_attribute(model, "reservation")
        PSI.add_variables!(container, PSI.ReservationVariable, devices, D())
    end

    PSI.add_parameters!(container, OutageTimeSeriesParameter, devices, model)
    PSI.initial_conditions!(container, devices, D())

    PSI.add_to_expression!(
        container,
        PSI.ActivePowerBalance,
        PSI.ActivePowerInVariable,
        devices,
        model,
        S,
    )
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerBalance,
        PSI.ActivePowerOutVariable,
        devices,
        model,
        S,
    )
    PSI.add_to_expression!(
        container,
        PSI.ReactivePowerBalance,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
    )
    PSI.add_feedforward_arguments!(container, model, devices)
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{St, D},
    ::Type{S},
) where {St <: PSY.Storage, D <: BookKeepingOutage, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(St, sys)

    PSI.add_constraints!(
        container,
        PSI.OutputActivePowerVariableLimitsConstraint,
        PSI.ActivePowerOutVariable,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        PSI.InputActivePowerVariableLimitsConstraint,
        PSI.ActivePowerInVariable,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        PSI.ReactivePowerVariableLimitsConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        PSI.EnergyCapacityConstraint,
        PSI.EnergyVariable,
        devices,
        model,
        S,
    )

    # Energy Balanace limits
    PSI.add_constraints!(container, PSI.EnergyBalanceConstraint, devices, model, S)

    PSI.add_constraints!(container, OutageUpperBoundConstraint, devices, model, S)

    PSI.add_feedforward_constraints!(container, model, devices)

    PSI.add_constraint_dual!(container, sys, model)
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{St, D},
    ::Type{S},
) where {St <: PSY.Storage, D <: BookKeepingOutage, S <: PM.AbstractActivePowerModel}
    devices = PSI.get_available_components(St, sys)

    PSI.add_variables!(container, PSI.ActivePowerInVariable, devices, D())
    PSI.add_variables!(container, PSI.ActivePowerOutVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergyVariable, devices, D())
    if PSI.get_attribute(model, "reservation")
        PSI.add_variables!(container, PSI.ReservationVariable, devices, D())
    end

    PSI.add_parameters!(container, OutageTimeSeriesParameter, devices, model)
    PSI.initial_conditions!(container, devices, D())

    PSI.add_to_expression!(
        container,
        PSI.ActivePowerBalance,
        PSI.ActivePowerInVariable,
        devices,
        model,
        S,
    )
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerBalance,
        PSI.ActivePowerOutVariable,
        devices,
        model,
        S,
    )
    PSI.add_feedforward_arguments!(container, model, devices)
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{St, D},
    ::Type{S},
) where {St <: PSY.Storage, D <: BookKeepingOutage, S <: PM.AbstractActivePowerModel}
    devices = PSI.get_available_components(St, sys)

    PSI.add_constraints!(
        container,
        PSI.OutputActivePowerVariableLimitsConstraint,
        PSI.ActivePowerOutVariable,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        PSI.InputActivePowerVariableLimitsConstraint,
        PSI.ActivePowerInVariable,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        PSI.EnergyCapacityConstraint,
        PSI.EnergyVariable,
        devices,
        model,
        S,
    )

    # Energy Balanace limits
    PSI.add_constraints!(container, PSI.EnergyBalanceConstraint, devices, model, S)

    PSI.add_constraints!(container, OutageUpperBoundConstraint, devices, model, S)

    PSI.add_feedforward_constraints!(container, model, devices)

    PSI.add_constraint_dual!(container, sys, model)
    return
end
