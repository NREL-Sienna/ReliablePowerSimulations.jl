abstract type AbstractThermalOutageDispatchFormulation <:
              PSI.AbstractThermalDispatchFormulation end
abstract type AbstractThermalOutageCommitmentFormulation <:
              PSI.AbstractStandardUnitCommitment end
struct ThermalStandardUCOutages <: AbstractThermalOutageCommitmentFormulation end
struct ThermalBasicUCOutages <: AbstractThermalOutageCommitmentFormulation end
struct ThermalDispatchOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalRampLimitedOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalNoMinOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalNoMinRampLimitedOutages <: AbstractThermalOutageDispatchFormulation end

struct HydroDispatchRunOfRiverOutage <: PSI.AbstractHydroDispatchFormulation end
struct HydroDispatchReservoirStorageOutage <: PSI.AbstractHydroReservoirFormulation end
struct HydroDispatchPumpedStoragewReservationOutage <: PSI.AbstractHydroReservoirFormulation end

struct BookKeepingOutage <: PSI.AbstractStorageFormulation end

struct RenewableDispatchOutages <: PSI.AbstractRenewableDispatchFormulation end
