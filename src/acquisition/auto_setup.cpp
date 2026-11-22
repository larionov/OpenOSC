#include "openosc/acquisition/auto_setup.hpp"
#include <algorithm>

namespace openosc::scope {
void AutoSetup::begin(const ScopeRequest &request,unsigned channel,uint32_t now) {
    original_=trial_=request;
    // The source: the trigger's own channel while it is enabled (the one the user triggers on, whatever
    // channel the keys select now), else the selected one, else the enabled one.
    const unsigned source=unsigned(request.trigger.channel);
    channel_=source<2 && request.frontend.enabled[source] ? source
           : channel<2 && request.frontend.enabled[channel] ? channel : request.frontend.enabled[0] ? 0 : 1;
    trial_.trigger.mode=TriggerMode::Auto;
    trial_.trigger.channel=Channel(channel_); trial_.trigger.edge=Edge::Rising; trial_.trigger.level=0;
    // Start independently of a previous20s/div or10ns/div window.
    // Trials keep the record budget: DataAccess captures them into its own trial area at a fixed
    // depth, so the record on show (and a HOLD to return to) stays untouched.
    trial_.timebaseNs=1000000;
    trial_.rollAuto=false;
    for (unsigned c=0;c<2;++c) if (trial_.frontend.enabled[c]) {
        trial_.frontend.range[c]=VoltageRanges::Count-1;
        trial_.frontend.position[c]=80;
    }
    state_=State::Searching; attempts_=0; hasSequence_=timing_=verified_=provisional_=counterRejected_=false;
    direction_[0]=direction_[1]=0;
    counterWait_=CounterWait::None;
    totalDeadline_=now+600000;
    changed(now);
}
bool AutoSetup::changed(uint32_t now) {
    // Each finite trial gets its actual capture duration plus2s for settling,
    // snapshot copy and foreground work. The longest supported window is200s.
    deadline_=now+recordMs()+2000;
    appliedAt_=now; if (counterWait_==CounterWait::Waiting) counterWait_=CounterWait::None;
    if (++attempts_>64) { state_=State::Timeout; return false; }
    return true;
}
void AutoSetup::tick(uint32_t now) {
    if (active() && (int32_t(now-deadline_)>=0 || int32_t(now-totalDeadline_)>=0)) state_=State::Timeout;
}
void AutoSetup::complete() {
    trial_.rollAuto=original_.rollAuto;
    state_=State::Complete;
}
bool AutoSetup::observe(const FrameView &frame,uint32_t now,const CounterSnapshot *counters) {
    tick(now);
    if (!active() || !frame.valid() || (hasSequence_ && sequence_==frame.metadata().sequence)) return false;
    const auto &metadata=frame.metadata();
    if (metadata.timebaseNs && metadata.timebaseNs!=trial_.timebaseNs) return false;
    for (unsigned c=0;c<2;++c) if (trial_.frontend.enabled[c]) {
        if (!metadata.channels[c].calibrated) { state_=State::Uncalibrated; return false; }
        if (metadata.channels[c].millivoltsPerDivision!=VoltageRanges::millivolts(trial_.frontend.range[c]) ||
            metadata.channels[c].coupling!=trial_.frontend.coupling[c]) return false;
    }
    sequence_=metadata.sequence; hasSequence_=true;
    SignalSummary summaries[2]; uint8_t usable=0; bool reversed=false;
    if (vertical(frame,summaries,usable,reversed)) {
        timing_=false;
        // The records disagree (direction_): a signal slower than the record. The new range stands
        // (this record's evidence) and the window is ten times longer, as for a record under two edges.
        if (reversed && trial_.timebaseNs<LongestTimebaseNs) {
            direction_[0]=direction_[1]=0;
            provisional_=false; // FCNT's timebase, if taken, is superseded by the longer window.
            return widen(now);
        }
        return changed(now);
    }
    if (!active()) return false;
    if (!usable) { state_=State::NoSignal; return false; }
    // The level and the edge bands come from the bulk of the codes: one spike must not lift the
    // middle above a square wave, while a train of narrow pulses stays the signal (SignalBands).
    SignalBand bands[2];
    SignalBands::measure(frame,bands);
    Periodicity periodic[2];
    for (unsigned c=0;c<2;++c) if ((usable&(1U<<c)) && bands[c].valid) periodic[c]=periodicity(frame,Channel(c),bands[c]);
    // The source stays the trigger's channel while it is usable and has a signal. The other one is
    // taken only for a flat source beside a channel with a span, or for edges without a period (noise,
    // an ADC floor with transients) beside a periodic channel. A source with fewer edges than a period
    // needs is a signal slower than this record, not noise: timing() widens the window for it, as a
    // faster channel beside it must not take the trigger over.
    const unsigned previousChannel=channel_;
    if (!(usable&(1U<<channel_))) channel_=usable&1 ? 0 : 1;
    const unsigned other=channel_^1;
    const auto noise=[&](unsigned c) { return !periodic[c].coherent && periodic[c].edges>=SlowEdges; };
    if ((usable&(1U<<other)) &&
        ((periodic[other].coherent && noise(channel_)) ||
         (summaries[channel_].maximum-summaries[channel_].minimum<4 && summaries[other].maximum-summaries[other].minimum>=4)))
        channel_=other;
    trial_.trigger.channel=Channel(channel_);
    const auto &band=bands[channel_];
    if (!band.valid) return false;
    const int16_t level=levelOf(band);
    const bool sourceChanged=channel_!=previousChannel;
    if (sourceChanged) { provisional_=counterRejected_=false; counterWait_=CounterWait::None; }
    if (!timing_ || sourceChanged) {
        timing_=true; verified_=false;
        if (trial_.trigger.level!=level || sourceChanged) { trial_.trigger.level=level; return changed(now); }
    }
    return timing(band,periodic[channel_],metadata.samplesPerSecond.value(),now,counters);
}
AutoSetup::Periodicity AutoSetup::periodicity(const FrameView &frame,Channel channel,const SignalBand &band) {
    // Edges between 30 % and 70 % of the band; the period is the one most of them agree on.
    EdgeScanner edges;
    edges.begin(band.low,band.high,EdgeScanner::laneBurst(frame.metadata()));
    edges.scan(frame,channel);
    Periodicity result;
    result.edges=edges.edges();
    result.coherent=edges.votedFrequency(frame.metadata().samplesPerSecond.value(),result.hertz);
    if (!result.coherent) result.hertz=0;
    return result;
}
int16_t AutoSetup::levelOf(const SignalBand &band) { return int16_t(std::clamp((int(band.low)+band.high)/2,-127,126)); }
} // namespace openosc::scope
