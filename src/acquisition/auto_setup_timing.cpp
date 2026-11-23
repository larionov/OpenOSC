#include "openosc/acquisition/auto_setup.hpp"
#include "openosc/acquisition/timebases.hpp"
#include <algorithm>
#include <cmath>

namespace openosc::scope {
uint64_t AutoSetup::timebaseFor(double hertz) {
    const double target=300000000.0/hertz;
    for (auto value:Timebases::Values) if (value>=target) return value;
    return Timebases::Values[sizeof(Timebases::Values)/sizeof(*Timebases::Values)-1];
}
bool AutoSetup::widenTenfold(uint32_t now) {
    const uint64_t target=trial_.timebaseNs*10;
    uint64_t next=LongestTimebaseNs;
    for (auto value:Timebases::Values) if (value>=target) { next=value; break; }
    setTimebase(next); provisional_=false;
    return changed(now);
}
bool AutoSetup::timing(const SignalBand &band,const Periodicity &raw,double sampleRate,uint32_t now,const CounterSnapshot *counters) {
    // Let a new hardware gate finish after changing the comparator level.
    if (counters && uint32_t(now-appliedAt_)<300) return false;
    double frequency=raw.hertz;
    if (provisional_) {
        // FCNT's timebase was taken without a raw period: this record, at that timebase, decides,
        // whatever the counter says now (it may have no reading at all). A period confirms it;
        // none means FCNT counted noise, comparator chatter or a DC level: back to where it was
        // taken, and FCNT is not taken again for this source.
        provisional_=false;
        if (!raw.coherent) {
            counterRejected_=true;
            if (trial_.timebaseNs!=fallbackTimebase_) { setTimebase(fallbackTimebase_); return changed(now); }
        }
    }
    if (counters && !counterRejected_) {
        const auto &hardware=counters->channels[channel_];
        const bool counted=(counters->enabled&(1U<<channel_)) && hardware.settled() && hardware.edges>=4 &&
                           hardware.hertz>0 && std::isfinite(hardware.hertz);
        // Edges but no period (aliased): FCNT decides; a completed wait per search and source, the deadline moved past it.
        if (!counted && !raw.coherent && raw.edges>=2 && (counters->enabled&(1U<<channel_)) && counterWait_!=CounterWait::Done) {
            const uint32_t end=now+CounterWaitMs+2*recordMs()+2000;
            if (counterWait_==CounterWait::None && int32_t(totalDeadline_-end)>0) {
                counterWait_=CounterWait::Waiting; counterWaitAt_=now;
                if (int32_t(end-deadline_)>0) deadline_=end;
            }
            if (counterWait_==CounterWait::Waiting) {
                if (uint32_t(now-counterWaitAt_)<CounterWaitMs) return false;
                counterWait_=CounterWait::Done;
            }
        }
        if (counted) {
            if (raw.coherent) {
                // A slow noisy comparator crossing can count several edges per actual period, and
                // a fast signal can alias in the ADC: FCNT is taken only if the raw period agrees
                // with it or with its folded alias at this capture's sample rate.
                const double folded=std::abs(std::remainder(hardware.hertz,sampleRate));
                const double tolerance=std::max(raw.hertz*.05,hardware.gateUs ? 3000000.0/hardware.gateUs : 15.0);
                if (std::abs(folded-raw.hertz)<=tolerance) frequency=hardware.hertz;
            } else {
                // No raw period here (a signal slower than the record, an aliased fast one, or
                // noise): FCNT's timebase, provisionally; its record must show the period.
                const uint64_t next=timebaseFor(hardware.hertz);
                if (next!=trial_.timebaseNs) {
                    provisional_=true; fallbackTimebase_=trial_.timebaseNs;
                    setTimebase(next);
                    return changed(now);
                }
                counterRejected_=true; // This record is at FCNT's timebase and shows no period.
            }
        }
    }
    if (frequency>0 && std::isfinite(frequency)) {
        const uint64_t next=timebaseFor(frequency);
        if (next!=trial_.timebaseNs) { setTimebase(next); return changed(now); }
    } else if (band.high-band.low>=4 && raw.edges<2 && canWiden()) {
        // Under two edges in the record: a slower signal, looked for in a ten times longer window.
        // Edges without a period (noise, an aperiodic signal) keep this timebase.
        return widenTenfold(now);
    }
    // The level was set on an earlier record, possibly at another timebase: checked once on this
    // one, and moved (one more trial) if the middle of its band is elsewhere.
    const int16_t level=levelOf(band);
    if (!verified_) {
        verified_=true;
        if (std::abs(level-trial_.trigger.level)>std::max(2,(band.high-band.low)/8)) { trial_.trigger.level=level; return changed(now); }
    }
    complete();
    return true;
}
} // namespace openosc::scope
