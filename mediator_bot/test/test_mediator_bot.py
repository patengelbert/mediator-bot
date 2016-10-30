import mediator_bot


class TestMediatorBot:
    def test_version(self):
        assert mediator_bot.__version__ >= "1.0.0"
