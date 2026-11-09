"""Check the registered current controls against both packaged Pilot layouts."""
import json
from pathlib import Path
import re
import unittest
MODULE = Path(__file__).resolve().parents[1] / 'psdk-overlay/samples/sample_c/module_sample/balloon_widget'
class WidgetLayoutTests(unittest.TestCase):
    def test_both_languages_match_packaged_handlers(self):
        source = (MODULE/'psdk_balloon_widget.c').read_text()
        definitions = dict(re.findall(r'^#define (BALLOON_WIDGET_\w+_INDEX) (\d+)$', source, re.MULTILINE))
        handlers = re.findall(r'\{(BALLOON_WIDGET_\w+_INDEX), DJI_WIDGET_TYPE_(BUTTON|SWITCH|LIST), BalloonWidget_SetValue', source)
        registered = {int(definitions[name]):kind.lower() for name,kind in handlers}
        self.assertEqual(len(handlers),26)
        schemas=[]
        for language in ('cn','en'):
            config=json.loads((MODULE/f'widget_file/{language}_big_screen/widget_config.json').read_text())
            main=config['main_interface']['widget_list'];settings=config['config_interface']['widget_list']
            self.assertEqual([item['widget_index'] for item in main],[50,55,51,81,82,83,84,58])
            self.assertEqual([item['widget_index'] for item in settings],[69,60,61,52,53,54,65,66,67,91,92,93,94,98,99,68])
            entries=main+settings
            by_id={item['widget_index']:item for item in entries}
            voice_input_name='语音输入任务' if language=='cn' else 'Voice task input'
            self.assertEqual(by_id[50]['widget_name'],voice_input_name)
            self.assertEqual(by_id[60]['widget_name'],voice_input_name)
            schema=[(item['widget_index'],item['widget_type']) for item in entries]
            self.assertEqual(len(schema),24)
            # v7 still accepts cached shortcuts for these two actions; the
            # current layout displays their existing settings-page aliases.
            self.assertEqual(dict(schema), {i:kind for i,kind in registered.items() if i not in (56,57)})
            for quick, setting in ((81,91),(82,92),(83,93)):
                self.assertEqual(
                    [item['item_name'] for item in by_id[quick]['list_item']],
                    [item['item_name'] for item in by_id[setting]['list_item']])
                self.assertTrue(all('icon_file_set' in item for item in by_id[quick]['list_item']))
            schemas.append(schema)
        self.assertEqual(schemas[0],schemas[1])
if __name__=='__main__':unittest.main()
