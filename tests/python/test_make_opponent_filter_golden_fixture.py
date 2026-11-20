"""The golden fixture orders events the way the app's tick loop does."""

from __future__ import annotations

from playground.opponent_prediction import make_opponent_filter_golden_fixture as fixture


def _coast(deliver: float) -> fixture.Event:
    return fixture.Event(fixture.KIND_COAST, deliver, coast_now_s=deliver)


def _measured(deliver: float) -> fixture.Event:
    return fixture.Event(fixture.KIND_MEASURED, deliver, shutter_stamp_s=deliver - 0.06)


def _empty(deliver: float) -> fixture.Event:
    return fixture.Event(fixture.KIND_EMPTY, deliver, shutter_stamp_s=deliver - 0.06)


def test_measurement_sorts_ahead_of_the_coast_tick_that_shares_its_delivery_time() -> None:
    events = [_coast(100.004), _coast(100.008), _measured(100.004), _empty(100.008)]

    ordered = fixture.order_events(events)

    assert [(e.kind, e.deliver) for e in ordered] == [
        (fixture.KIND_MEASURED, 100.004),
        (fixture.KIND_COAST, 100.004),
        (fixture.KIND_EMPTY, 100.008),
        (fixture.KIND_COAST, 100.008),
    ]


def test_events_order_by_delivery_time_first() -> None:
    ordered = fixture.order_events(
        [_coast(100.0), _measured(100.5), _coast(100.5), _measured(99.9)]
    )

    assert [e.deliver for e in ordered] == [99.9, 100.0, 100.5, 100.5]


def test_delivery_times_equal_to_nine_decimals_count_as_shared() -> None:
    ordered = fixture.order_events([_coast(100.0), _measured(100.0 + 1e-12)])

    assert ordered[0].kind == fixture.KIND_MEASURED


def test_scenario_never_puts_a_coast_ahead_of_a_measurement_it_ties_with() -> None:
    events = fixture.scenario()
    for before, after in zip(events, events[1:]):
        if round(before.deliver, 9) == round(after.deliver, 9):
            assert not (before.kind == fixture.KIND_COAST and after.kind != fixture.KIND_COAST)
