from pathlib import Path


REPOSITORY = Path(__file__).parents[2]
SCRIPT = REPOSITORY / "tools_demo" / "scripts" / "clean_launch.sh"
DEBUG_START = REPOSITORY.parents[2] / "scripts" / "05_start_debug.sh"


def test_clean_launch_uses_current_workspace_and_debug_gui():
    """The new workspace must launch the bounded Gazebo GUI client."""
    source = SCRIPT.read_text(encoding="utf-8")
    assert "cd /home/ros/dev_ws" not in source
    assert "debug_gui:" in source and "debug_gui_mode" in source
    assert "YZBOT_GAZEBO_GUI_RENDER_RATE" in source
    assert "YZBOT_GAZEBO_GUI_WIDTH" in source
    assert "YZBOT_GAZEBO_GUI_HEIGHT" in source


def test_debug_entrypoint_selects_bounded_gui():
    """The portable debug entrypoint must opt into the GUI plugin."""
    source = DEBUG_START.read_text(encoding="utf-8")
    assert "debug_gui:=True" in source


def test_debug_plugin_re_asserts_bounded_viewport():
    """Mutter can re-maximize the window, so geometry is re-asserted."""
    source = (
        REPOSITORY / "gazebo_debug_gui" / "src" / "gazebo_debug_gui_plugin.cpp"
    ).read_text(encoding="utf-8")
    assert "geometryAttempts < 20" in source
    assert "window->resize(this->windowWidth, this->windowHeight);" in source
    assert "this->geometryAttempts >= 20" in source
