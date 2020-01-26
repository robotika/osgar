""""
  Extract robot positions from Ignition SQLite log file
"""
import sqlite3
import struct


def unpack_protobuf(data, depth=0, prefix=None, verbose=False):
    if depth >= 3:
        return
    if prefix is None:
        prefix = []
    if verbose:
        print(prefix, data[:100].hex())
    pos = 0
    while pos < len(data):
        c = data[pos]
        field_number = c >> 3
        wire_type = c & 0x7
        if verbose:
            print(pos, field_number, wire_type)
        pos += 1
        if wire_type == 0:  # variant
            start = pos
            while data[pos] & 0x80 == 0x80:
                pos += 1
            pos += 1
#            print('Var', field_number, data[start:pos].hex())
        elif wire_type == 1:  # 64bit
            d = struct.unpack('<d', data[pos:pos+8])[0]
#            i = struct.unpack('<q', data[:8])[0]
            if prefix == [2, 4]:  # xyz
                print('double', prefix, field_number, d)
#            print('int', prefix, field_number, i)
            pos += 8
        elif wire_type == 2:
            size = data[pos]
            pos += 1

            name = data[pos:pos + size]
            if verbose:
                print(name)
            if name in [b'base_link', b'front_left_wheel', b'front_right_wheel', b'rear_left_wheel', b'rear_right_wheel']:
#                print('terminated', name)
                return
            if prefix == [2] and field_number == 2:
                print(prefix, field_number, wire_type, size, data[pos:pos + size])
            key = prefix + [field_number]
            if field_number in [2, 4, 5]:
#                print('Key', key)
                if key != [2, 2]:
                    unpack_protobuf(data[pos:pos + size], depth=depth+1, prefix=key)
            pos += size
        else:
            assert False, wire_type


def extract_poses(filename):
    con = sqlite3.connect(filename)
    cursor = con.cursor()

    # overview of available tables
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = cursor.fetchall()
    for table in tables:
        cursor.execute("SELECT sql FROM sqlite_master WHERE type='table' and name='%s';" % table)
#        print(cursor.fetchall()[0][0])
#        print()

    # start time?
    cursor.execute("SELECT * FROM migrations;")
    print('Start record:', cursor.fetchall())

    cursor.execute("SELECT * FROM topics;")
    topics = cursor.fetchall()
    print(topics)

    cursor.execute(r"SELECT id FROM topics where name LIKE '%/dynamic_pose/info';")
    result = cursor.fetchall()
    dynamic_topic_id = result[0][0]

    cursor.execute("SELECT * FROM messages;")
    for i, row in enumerate(cursor):
        index, time_ns, topic_id, data = row
        if topic_id == dynamic_topic_id:  # '/world/urban_circuit_practice_03/dynamic_pose/info'
#            print(data[:100].hex())
            unpack_protobuf(data)
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
        if i > 100:
            break


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ignfile', help='Ignition file from simulation (state.tlog)')
    args = parser.parse_args()
    extract_poses(args.ignfile)


# vim: expandtab sw=4 ts=4
