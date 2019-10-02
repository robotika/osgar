""""
  Extract robot positions from Ignition SQLite log file
"""
import sqlite3


def extract_poses(filename):
    con = sqlite3.connect(filename)
    cursor = con.cursor()

    # overview of available tables
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = cursor.fetchall()
    for table in tables:
        cursor.execute("SELECT sql FROM sqlite_master WHERE type='table' and name='%s';" % table)
        print(cursor.fetchall()[0][0])
        print()

    # start time?
    cursor.execute("SELECT * FROM migrations;")
    print('Start record:', cursor.fetchall())

    cursor.execute("SELECT * FROM topics;")
    topics = cursor.fetchall()
    print(topics)

    cursor.execute("SELECT * FROM messages;")
    for i, row in enumerate(cursor):
        index, time_ns, topic_id, data = row
        if topic_id == 1:
            print(index, data, len(data))
        if i > 20:
            break


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ignfile', help='Ignition file from simulation (state.tlog)')
    args = parser.parse_args()
    extract_poses(args.ignfile)


# vim: expandtab sw=4 ts=4
