#!/usr/bin/env python

#   Copyright 2015-2016 Scott Bezek and the splitflap contributors
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

import logging
import os
import re
import subprocess
import sys
import tempfile
import time

from contextlib import contextmanager

repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ci_root = os.path.join(repo_root, 'ci')
sys.path.append(ci_root)

from util import file_util, rev_info

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


def get_versioned_contents(filename, release_search_prefix):
    with open(filename, 'r') as f:
        original_contents = f.read()
        date = rev_info.git_date()
        date_long = rev_info.git_date(short=False)
        rev = rev_info.git_short_rev()
        logger.info('Replacing placeholders with %s and %s' % (date, rev))
        release_version = 'v#.#'
        if release_search_prefix:
            tag_version = rev_info.git_release_version(release_search_prefix)
            if tag_version:
                logger.info(f'Tag found: {tag_version}')
                release_version = tag_version
            else:
                logger.info(f'No tagged release found with prefix {release_search_prefix!r}')
        return original_contents, original_contents \
            .replace('Date ""', 'Date "%s"' % date_long) \
            .replace('DATE: YYYY-MM-DD HH:MM:SS TZ', 'DATE: %s' % date_long) \
            .replace('${COMMIT_DATE_LONG}', date_long) \
            .replace('DATE: YYYY-MM-DD', 'DATE: %s' % date) \
            .replace('${COMMIT_DATE}', date) \
            .replace('Rev ""', 'Rev "%s"' % rev) \
            .replace('COMMIT: deadbeef', 'COMMIT: %s' % rev) \
            .replace('${COMMIT_HASH}', rev) \
            .replace('v#.#', release_version) \
            .replace('${RELEASE_VERSION}', release_version)


@contextmanager
def versioned_file(filename, release_search_prefix):
    original_contents, versioned_contents = get_versioned_contents(filename, release_search_prefix)
    with open(filename, 'w') as temp_schematic:
        logger.debug('Writing to %s', filename)
        temp_schematic.write(versioned_contents)
    try:
        yield
    finally:
        with open(filename, 'w') as temp_schematic:
            logger.debug('Restoring %s', filename)
            temp_schematic.write(original_contents)
