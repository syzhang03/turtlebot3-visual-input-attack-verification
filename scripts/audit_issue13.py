"""Recompute evaluation outcomes from retained evidence without repeating trials."""
import argparse
import json
from pathlib import Path

from issue13_metrics import analyze


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder')
    args = parser.parse_args()
    folder = Path(args.folder)
    original = json.loads((folder/'runs.json').read_text())
    status = json.loads((folder/'status.json').read_text())
    if len(original) != status['planned'] or not status['source_unchanged']:
        raise ValueError('Batch incomplete or unfrozen')
    records = []
    for trial in original:
        row = {k: trial[k] for k in ('trial','group','controller','input','empty','repeat','exit_code')}
        row.update(valid=False, collection_valid=trial['valid'], collection_warning=trial.get('error'))
        if row['exit_code'] == 0:
            try:
                row.update(analyze(folder/row['trial'],row['input'],row['empty'],row['controller']=='fusion'))
            except (ValueError, OSError, StopIteration, KeyError) as error:
                row['error'] = str(error)
        else:
            row['error'] = trial.get('error')
        records.append(row)
    (folder/'analysis-runs.json').write_text(json.dumps(records,indent=2)+'\n')
    result = dict(status, valid=sum(r['valid'] for r in records),
                  collection_valid=status['valid'],
                  recovered=[r['trial'] for r in records if r['valid'] and not r['collection_valid']],
                  pending=[r['trial'] for r in records if not r['valid']])
    (folder/'analysis-status.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
