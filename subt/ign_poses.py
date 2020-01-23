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
        if topic_id == 2:  # '/world/urban_circuit_practice_03/dynamic_pose/info'
            print(data[:10].hex())
#            print(index, data, len(data))
#           print(index, len(data))
#        if b'A60F900L' in data:  #i > 100:
#        if b'E180F200F700L' in data:  #i > 100:
#            assert b'\x08A60F900L' in data, data
#            print(data[:30])
#            print(data)
#            break
            """
            print('----------------')
            index = 0
#            while index < len(data):
            for j in range(10):
                size = data[index]
                index += 1
                print(size, data[index:index + size])
                index += size
                print(index, len(data))
                """
        if i > 100000:
            break


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ignfile', help='Ignition file from simulation (state.tlog)')
    args = parser.parse_args()
    extract_poses(args.ignfile)


# vim: expandtab sw=4 ts=4
