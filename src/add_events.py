if __name__ == "__main__":

    import json
    import rospy

    from strawberry_dialogue_knowrob.msg import QueryType, OntologyType
    from strawberry_dialogue_knowrob.srv import Query, QueryRequest

    from event_definition import EVENT_DEFINITIONS

    kb_query_client = rospy.ServiceProxy("/strawberry/kb/query", Query)

    for e in EVENT_DEFINITIONS:
        req = QueryRequest()
        req.query_type.value = QueryType.CREATE
        req.ontology_type.value = OntologyType.EVENTDEFINITION
        req.parameters.payload = json.dumps(e)

        res = kb_query_client(req)

        print(res)

    print("DONE")
