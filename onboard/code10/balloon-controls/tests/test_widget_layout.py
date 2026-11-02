import json
from pathlib import Path
import re
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'psdk-overlay/samples/sample_c/module_sample/balloon_widget'


class WidgetLayoutTests(unittest.TestCase):
    def test_both_languages_match_packaged_handlers(self):
        source = (MODULE / 'psdk_balloon_widget.c').read_text()
        definitions = dict(re.findall(r'^#define (BALLOON_WIDGET_\w+_INDEX) (\d+)$', source, re.MULTILINE))
        handlers = re.findall(r'\{(BALLOON_WIDGET_\w+_INDEX), DJI_WIDGET_TYPE_(BUTTON|SWITCH), BalloonWidget_SetValue', source)
        registered = {int(definitions[name]): kind.lower() for name, kind in handlers}
        self.assertEqual(set(registered), set(range(18, 30)))
        schemas = []
        for language in ('cn', 'en'):
            config = json.loads((MODULE / f'widget_file/{language}_big_screen/widget_config.json').read_text())
            main = config['main_interface']['widget_list']
            settings = config['config_interface']['widget_list']
            self.assertEqual([item['widget_index'] for item in main], list(range(18, 24)))
            self.assertEqual([item['widget_index'] for item in settings], [24, 26, 27, 28, 29, 25])
            entries = main + settings
            self.assertEqual(len(entries), 12)
            schema = [(item['widget_index'], item['widget_type']) for item in entries]
            self.assertEqual(dict(schema), registered)
            schemas.append(schema)
        self.assertEqual(schemas[0], schemas[1])


if __name__ == '__main__':
    unittest.main()
