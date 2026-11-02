"""Ground review and stopped-run maintenance CLI. No automatic labeling or deletion."""
import argparse
from contextlib import ExitStack
import json
import sys

from .hardcase import dataset


def parser():
    result = argparse.ArgumentParser(description='Validate candidate runs and export pinned human annotations. No automatic labeling.')
    commands = result.add_subparsers(dest='command', required=True)
    for name in ('retention-status', 'prepare-transfer', 'recover-run', 'cleanup-run'):
        command = commands.add_parser(name)
        command.add_argument('--data-root', required=True, help='Aircraft capture data root shared by all runs')
        if name != 'retention-status':
            command.add_argument('--run-id', required=True, help='Exact stopped acquisition run ID')
        if name == 'prepare-transfer':
            command.add_argument('--output', required=True, help='New manifest JSON file outside the data root')
        elif name == 'cleanup-run':
            command.add_argument('--receipt', required=True, help='Completion receipt from verified downloaded raw copy')
            command.add_argument('--execute', action='store_true', help='Delete this confirmed batch; default only previews')
    confirm = commands.add_parser('confirm-processed', help='Explicitly confirm successful processing after verifying a downloaded raw copy')
    confirm.add_argument('--run', required=True, help='Unmodified downloaded raw run directory')
    confirm.add_argument('--manifest', required=True, help='Frozen transfer manifest exported from the aircraft')
    confirm.add_argument('--processed-by', required=True, help='Operator explicitly confirming processing success')
    confirm.add_argument('--processing-note', required=True, help='Completed processing and retained output reference')
    confirm.add_argument('--output', required=True, help='New completion receipt JSON outside the downloaded run')
    for name in ('validate-run', 'export-review-manifest', 'import-human-reviews', 'export-yolo-dataset', 'compare-selections'):
        command = commands.add_parser(name)
        command.add_argument('--run', action='append', required=True, help='Run directory; repeat for independent runs')
        command.add_argument('--dry-run', action='store_true', help='Validate and print a report without writing any file/database')
        command.add_argument('--output', help='New report/export directory, outside all source runs')
        if name in ('import-human-reviews', 'export-yolo-dataset'):
            command.add_argument('--class-map', required=True, help='JSON {version,names:{"0":"class name"}}')
        if name == 'import-human-reviews':
            command.add_argument('--reviews', required=True, help='JSON list of explicit human reviews')
            command.add_argument('--ground-copy', action='store_true', help='Permit annotation writes to a verified relocated copy; acquisition writes stay forbidden')
        if name == 'export-yolo-dataset':
            command.add_argument('--review-ids', required=True, help='JSON list of pinned review IDs')
            command.add_argument('--assignments', help='JSON sample_id -> train/val/test')
            command.add_argument('--frozen-test-ids', help='JSON list of sample IDs frozen before selection')
            command.add_argument('--seed', type=int, default=0)
        if name in ('export-yolo-dataset', 'compare-selections'):
            command.add_argument('--existing-split', help='Full frozen split manifest for the complete sample pool; grouping and assignments are preserved')
            command.add_argument('--dhash-threshold', type=int, help='Optional near-duplicate dHash Hamming threshold 0..64')
            command.add_argument('--manual-groups', help='JSON sample_id -> scene/near-duplicate group')
        if name == 'compare-selections':
            command.add_argument('--budget', type=int, required=True)
            command.add_argument('--seeds', nargs='+', type=int, required=True, help='At least three fixed seeds')
            command.add_argument('--d0-hashes', required=True, help='JSON list of original dataset image SHA256s')
            command.add_argument('--frozen-test-ids', required=True)
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command in ('retention-status', 'prepare-transfer', 'confirm-processed', 'recover-run', 'cleanup-run'):
            from .hardcase import transfer
            if args.command == 'retention-status':
                report = transfer.retention_status(args.data_root)
            elif args.command == 'prepare-transfer':
                report = transfer.prepare_transfer(args.data_root, args.run_id, args.output)
            elif args.command == 'confirm-processed':
                report = transfer.confirm_processed(args.run, args.manifest, args.output,
                    processed_by=args.processed_by, processing_note=args.processing_note)
            elif args.command == 'recover-run':
                report = transfer.recover_run(args.data_root, args.run_id)
            else:
                report = transfer.cleanup_run(args.data_root, args.run_id, args.receipt, execute=args.execute)
            print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
            return 0
        from .hardcase.storage import RunStore
        with ExitStack() as stack:
            writable = args.command == 'import-human-reviews' and not args.dry_run
            if args.command == 'import-human-reviews' and len(args.run) != 1:
                raise ValueError('import one run per atomic batch')
            stores = [stack.enter_context(RunStore.open(path, read_only=not writable,
                       **({'ground_copy': True} if getattr(args, 'ground_copy', False) and writable else {})))
                      for path in args.run]
            roots = [store.root for store in stores]
            if args.output:
                dataset.new_output(args.output, roots)
            def optional(name, default):
                value = getattr(args, name, None)
                return dataset.read_json(value) if value else default
            if args.command == 'validate-run':
                report = {'runs': [dataset.validate_run(s) for s in stores]}
            elif args.command == 'export-review-manifest':
                if not args.output:
                    raise ValueError('--output is required')
                report = dataset.export_review_manifest(stores, args.output, args.dry_run)
            elif args.command == 'import-human-reviews':
                report = dataset.import_human_reviews(stores[0], dataset.read_json(args.reviews),
                                   dataset.read_json(args.class_map), args.dry_run)
            elif args.command == 'export-yolo-dataset':
                if not args.output:
                    raise ValueError('--output is required')
                existing = optional('existing_split', None)
                report = dataset.export_yolo_dataset(stores, dataset.read_json(args.review_ids),
                    dataset.read_json(args.class_map), args.output, assignments=optional('assignments', None),
                    frozen_test_ids=optional('frozen_test_ids', []), seed=args.seed,
                    manual_groups=optional('manual_groups', None), dhash_threshold=args.dhash_threshold,
                    existing_split=existing, dry_run=args.dry_run)
            else:
                samples, owners = dataset._collect(stores)
                existing = optional('existing_split', None)
                manual, threshold = dataset._grouping_config(optional('manual_groups', None), args.dhash_threshold, existing)
                report = dataset.compare_selections(samples, budget=args.budget, seeds=args.seeds,
                    d0_hashes=dataset.read_json(args.d0_hashes), frozen_test_ids=dataset.read_json(args.frozen_test_ids),
                    manual_groups=manual, dhashes=dataset._pool_dhashes(samples, owners, threshold),
                    dhash_threshold=threshold, existing_split=existing)
            if args.output and args.command in ('validate-run', 'import-human-reviews', 'compare-selections'):
                dataset.write_report(args.output, report, roots, args.dry_run)
            print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
            return 0
    except (ValueError, OSError, KeyError, TypeError, RuntimeError) as exc:
        print('hardcase: ' + str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
