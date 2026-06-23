from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navside.app import NavSideApp


def main():
    app = NavSideApp.from_config(str(Path(__file__).resolve().parents[1] / 'config' / 'nav.yaml'))
    result = app.demo_tick(control=True)
    assert result is not None
    ctrl = result['control']
    assert ctrl['final_cmd'].shape == (3,)
    assert ctrl['final_cmd'][1] == 0.0
    print('control smoke ok')


if __name__ == '__main__':
    main()
