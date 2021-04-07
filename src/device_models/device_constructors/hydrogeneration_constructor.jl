function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{H, HydroDispatchRunOfRiverOutage},
    ::Type{S},
) where {H <: PSY.HydroGen, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(H, sys)

    if !PSI.validate_available_devices(H, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerVariable,
        devices,
        HydroDispatchRunOfRiverOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReactivePowerVariable,
        devices,
        HydroDispatchRunOfRiverOutage(),
    )

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
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(optimization_container, devices, model, S, nothing)

    return
end

"""
Construct model for HydroGen with RunOfRiver Dispatch Formulation
with only Active Power.
"""
function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{H, HydroDispatchRunOfRiverOutage},
    ::Type{S},
) where {H <: PSY.HydroGen, S <: PM.AbstractActivePowerModel}
    devices = PSI.get_available_components(H, sys)

    if !PSI.validate_available_devices(H, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerVariable,
        devices,
        HydroDispatchRunOfRiverOutage(),
    )

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
    PSI.cost_function!(optimization_container, devices, model, S, nothing)

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{H, HydroDispatchReservoirStorageOutage},
    ::Type{S},
) where {H <: PSY.HydroEnergyReservoir, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(H, sys)

    if !PSI.validate_available_devices(H, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerVariable,
        devices,
        HydroDispatchReservoirStorageOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReactivePowerVariable,
        devices,
        HydroDispatchReservoirStorageOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyVariable,
        devices,
        HydroDispatchReservoirStorageOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.SpillageVariable,
        devices,
        HydroDispatchReservoirStorageOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyShortageVariable,
        devices,
        HydroDispatchReservoirStorageOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergySurplusVariable,
        devices,
        HydroDispatchReservoirStorageOutage(),
    )

    # Initial Conditions
    PSI.storage_energy_initial_condition!(
        optimization_container,
        devices,
        HydroDispatchReservoirStorageOutage(),
    )
    # Energy Balance Constraint
    PSI.add_constraints!(
        optimization_container,
        PSI.EnergyBalanceConstraint,
        PSI.EnergyVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.energy_target_constraint!(
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
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(optimization_container, devices, model, S, nothing)

    return
end

"""
Construct model for HydroGen with ReservoirStorage Dispatch Formulation
with only Active Power
"""
function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{H, HydroDispatchReservoirStorageOutage},
    ::Type{S},
) where {H <: PSY.HydroEnergyReservoir, S <: PM.AbstractActivePowerModel}
    devices = PSI.get_available_components(H, sys)

    if !PSI.validate_available_devices(H, devices)
        return
    end

    # Variables

    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerVariable,
        devices,
        HydroDispatchReservoirStorageOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyVariable,
        devices,
        HydroDispatchReservoirStorageOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.SpillageVariable,
        devices,
        HydroDispatchReservoirStorageOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyShortageVariable,
        devices,
        HydroDispatchReservoirStorageOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergySurplusVariable,
        devices,
        HydroDispatchReservoirStorageOutage(),
    )

    # Initial Conditions
    PSI.storage_energy_initial_condition!(
        optimization_container,
        devices,
        HydroDispatchReservoirStorageOutage(),
    )
    # Energy Balance Constraint
    PSI.add_constraints!(
        optimization_container,
        PSI.EnergyBalanceConstraint,
        PSI.EnergyVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.energy_target_constraint!(
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
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(optimization_container, devices, model, S, nothing)

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{H, HydroDispatchPumpedStoragewReservationOutage},
    ::Type{S},
) where {H <: PSY.HydroPumpedStorage, S <: PM.AbstractActivePowerModel}
    devices = PSI.get_available_components(H, sys)

    if !PSI.validate_available_devices(H, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerInVariable,
        devices,
        HydroDispatchPumpedStoragewReservationOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerOutVariable,
        devices,
        HydroDispatchPumpedStoragewReservationOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyVariableUp,
        devices,
        HydroDispatchPumpedStoragewReservationOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyVariableDown,
        devices,
        HydroDispatchPumpedStoragewReservationOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.SpillageVariable,
        devices,
        HydroDispatchPumpedStoragewReservationOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReserveVariable,
        devices,
        HydroDispatchPumpedStoragewReservationOutage(),
    )

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerOutVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerInVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    # Initial Conditions
    PSI.storage_energy_initial_condition!(
        optimization_container,
        devices,
        HydroDispatchPumpedStoragewReservationOutage(),
    )

    # Energy Balanace limits
    PSI.add_constraints!(
        optimization_container,
        PSI.EnergyBalanceConstraint,
        PSI.EnergyVariableUp,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.add_constraints!(
        optimization_container,
        PSI.EnergyBalanceConstraint,
        PSI.EnergyVariableDown,
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
    PSI.cost_function!(optimization_container, devices, model, S, nothing)

    return
end
