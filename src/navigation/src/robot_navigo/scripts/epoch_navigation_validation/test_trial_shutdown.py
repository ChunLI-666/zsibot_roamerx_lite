from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from run_stack import Trial


def test_plant_finalizes_even_when_process_wait_fails(monkeypatch):
    fixture=SimpleNamespace(finish=Mock())
    trial=Trial(None,fixture)
    process=SimpleNamespace(pid=1,poll=lambda:None,wait=Mock(side_effect=RuntimeError('wait failed')))
    trial.processes=[process]
    monkeypatch.setattr('run_stack.os.killpg',lambda *args:None)
    with pytest.raises(RuntimeError,match='wait failed'):
        trial.close()
    fixture.finish.assert_called_once()


def test_process_exit_race_still_finalizes_plant(monkeypatch):
    fixture=SimpleNamespace(finish=Mock())
    trial=Trial(None,fixture)
    trial.processes=[SimpleNamespace(pid=1,poll=lambda:None)]
    def missing(*args):raise ProcessLookupError()
    monkeypatch.setattr('run_stack.os.killpg',missing)
    trial.close()
    fixture.finish.assert_called_once()
