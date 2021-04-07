function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{St, BookKeepingwReservationOutage},
    ::Type{S},
) where {St <: PSY.Storage, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(St, sys)

    if !PSI.validate_available_devices(St, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerInVariable,
        devices,
        BookKeepingwReservationOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerOutVariable,
        devices,
        BookKeepingwReservationOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReactivePowerVariable,
        devices,
        BookKeepingwReservationOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyVariable,
        devices,
        BookKeepingwReservationOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReserveVariable,
        devices,
        BookKeepingwReservationOutage(),
    )

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, BookKeepingwReservationOutage())

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
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.energy_capacity_constraints!(
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

    # Energy Balanace limits
    PSI.energy_balance_constraint!(
        optimization_container,
        devices,
        model.formulation,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{St, BookKeepingwReservationOutage},
    ::Type{S},
) where {St <: PSY.Storage, S <: PM.AbstractActivePowerModel}
    devices = PSI.get_available_components(St, sys)

    if !PSI.validate_available_devices(St, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerInVariable,
        devices,
        BookKeepingwReservationOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerOutVariable,
        devices,
        BookKeepingwReservationOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyVariable,
        devices,
        BookKeepingwReservationOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReserveVariable,
        devices,
        BookKeepingwReservationOutage(),
    )

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, BookKeepingwReservationOutage())

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
    PSI.energy_capacity_constraints!(
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

    # Energy Balanace limits
    PSI.energy_balance_constraint!(
        optimization_container,
        devices,
        model.formulation,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{St, EndOfPeriodEnergyTargetOutage},
    ::Type{S},
) where {St <: PSY.Storage, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(St, sys)

    if !PSI.validate_available_devices(St, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerInVariable,
        devices,
        EndOfPeriodEnergyTargetOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerOutVariable,
        devices,
        EndOfPeriodEnergyTargetOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReactivePowerVariable,
        devices,
        EndOfPeriodEnergyTargetOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyVariable,
        devices,
        EndOfPeriodEnergyTargetOutage(),
    )

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, EndOfPeriodEnergyTargetOutage())

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
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.energy_capacity_constraints!(
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

    # Energy Balanace limits
    PSI.energy_balance_constraint!(
        optimization_container,
        devices,
        model.formulation,
        S,
        PSI.get_feedforward(model),
    )

    # Cost Function
    PSI.cost_function!(optimization_container, devices, model, S, PSI.get_feedforward(model))

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{St, EndOfPeriodEnergyTargetOutage},
    ::Type{S},
) where {St <: PSY.Storage, S <: PM.AbstractActivePowerModel}
    devices = PSI.get_available_components(St, sys)

    if !PSI.validate_available_devices(St, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerInVariable,
        devices,
        EndOfPeriodEnergyTargetOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerOutVariable,
        devices,
        EndOfPeriodEnergyTargetOutage(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyVariable,
        devices,
        EndOfPeriodEnergyTargetOutage(),
    )

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, EndOfPeriodEnergyTargetOutage())

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
    PSI.energy_capacity_constraints!(
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

    # Energy Balanace limits
    PSI.energy_balance_constraint!(
        optimization_container,
        devices,
        model.formulation,
        S,
        PSI.get_feedforward(model),
    )

    # Cost Function
    PSI.cost_function!(optimization_container, devices, model, S, PSI.get_feedforward(model))

    return
end
