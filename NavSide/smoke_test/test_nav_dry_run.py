from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navside.app import NavSideApp


def main():
    app = NavSideApp.from_config(str(Path(__file__).resolve().parents[1] / 'config' / 'nav.yaml'))
    result = app.demo_tick(control=False)
    assert result is not None
    assert tuple(result['diag']['depth_feature_shape'].tolist()) == (2560,)
    assert tuple(result['diag']['obs_shape'].tolist()) == (1, 2576)
    print('dry-run smoke ok')


if __name__ == '__main__':
    main()
