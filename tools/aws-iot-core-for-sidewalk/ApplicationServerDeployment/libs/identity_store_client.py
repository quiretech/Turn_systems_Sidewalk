# Copyright 2023 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0

import boto3
from botocore.exceptions import ClientError

from libs.user import User
from libs.utils import *


class IdentityStoreClient:
    """
    Provides Identity Store client with methods necessary to deploy/delete stack.

    Attributes
    ----------
        _client: botocore.client.IdentityStore
            Client to AWS Identity Store.
        _store_id: str
            Id of the identity store.
    """

    def __init__(self, session: boto3.Session, store_id: str):
        self._client = session.client(service_name='identitystore')
        self._store_id = store_id

    # -------
    # Deploy
    # -------
    def create_group(self, group_name: str) -> str:
        """
        Creates users' group.
        
        :param group_name:  Name of the group to be created.
        :return:            Id of created users' group.
        """
        group_id = ''
        try:
            log_info(f'Creating {group_name} group in IAM Identity Center...')
            response = self._client.create_group(
                IdentityStoreId=self._store_id,
                DisplayName=group_name,
                Description='Groups SidewalkGrafanaApplication dashboard viewers.'
            )
            eval_client_response(response, 'Group created.')
            group_id = response['GroupId']
        except ClientError as e:
            if e.response['Error']['Code'] == 'ConflictException':
                log_info(f'{group_name} already exists. Getting its ID...')
                response = self._client.get_group_id(
                    IdentityStoreId=self._store_id,
                    AlternateIdentifier={
                        'UniqueAttribute': {
                            'AttributePath': 'displayName',
                            'AttributeValue': group_name
                        }
                    }
                )
                eval_client_response(response, 'Group ID read properly.')
                group_id = response['GroupId']
            else:
                terminate(f'Group not created: {e}.', ErrCode.EXCEPTION)
        return group_id

    def create_user(self, user: User) -> str:
        """
        Creates user.

        :param user:    User object representing user to be created.
        :return:        Id of created user.
        """
        user_id = ''
        try:
            log_info(f'Creating {user.username} user in IAM Identity Center...')
            response = self._client.create_user(
                IdentityStoreId=self._store_id,
                UserName=user.username,
                Name={
                    'FamilyName': user.lastname,
                    'GivenName': user.firstname
                },
                DisplayName=user.username
            )
            eval_client_response(response, 'User created.')
            user_id = response['UserId']
        except ClientError as e:
            if e.response['Error']['Code'] == 'ConflictException':
                log_info(f'{user.username} already exists. Getting its ID...')
                response = self._client.get_user_id(
                    IdentityStoreId=self._store_id,
                    AlternateIdentifier={
                        'UniqueAttribute': {
                            'AttributePath': 'username',
                            'AttributeValue': user.username
                        }
                    }
                )
                eval_client_response(response, 'User ID read properly.')
                user_id = response['UserId']
            else:
                terminate(f'User not created: {e}.', ErrCode.EXCEPTION)
        return user_id

    def add_user_to_group(self, user: User, group_id: str):
        """
        Adds user to the group.

        :param user:        User object representing user to be added to the group.
        :param group_id:    Id of the already created group.
        """
        try:
            log_info(f'Adding {user.username} to the group...')
            response = self._client.create_group_membership(
                GroupId=group_id,
                IdentityStoreId=self._store_id,
                MemberId={
                    'UserId': user.id
                }
            )
            eval_client_response(response, 'User added to the group.')
        except ClientError as e:
            if e.response['Error']['Code'] == 'ConflictException':
                log_success(f'{user.username} is already member of the group.')
            else:
                terminate(f'Unable to add user to the group: {e}.', ErrCode.EXCEPTION)

    # -------
    # Delete
    # -------
    def delete_group(self, group_name: str):
        """
        Deletes user's group.
        
        :param group_name:  Name of the group to be deleted.
        """
        try:
            log_info(f'Deleting {group_name} group from IAM Identity Center...')
            response = self._client.get_group_id(
                IdentityStoreId=self._store_id,
                AlternateIdentifier={
                    'UniqueAttribute': {
                        'AttributePath': 'displayName',
                        'AttributeValue': group_name
                    }
                }
            )
            group_id = response.get('GroupId')
            response = self._client.delete_group(
                IdentityStoreId=self._store_id,
                GroupId=group_id
            )
            eval_client_response(response, 'Group deleted.')
        except ClientError as e:
            if e.response['Error']['Code'] == 'ResourceNotFoundException':
                log_success(f'{group_name} doesn\'t exist, skipping.')
            else:
                terminate(f'{group_name} deletion failed: {e}.', ErrCode.EXCEPTION)

    def delete_user(self, user: User):
        """
        Deletes user.

        :param user:    User object representing user to be deleted.
        """
        try:
            log_info(f'Deleting {user.username} user from IAM Identity Center...')
            response = self._client.get_user_id(
                IdentityStoreId=self._store_id,
                AlternateIdentifier={
                    'UniqueAttribute': {
                        'AttributePath': 'username',
                        'AttributeValue': user.username
                    }
                }
            )
            user_id = response.get('UserId')
            response = self._client.delete_user(
                IdentityStoreId=self._store_id,
                UserId=user_id
            )
            eval_client_response(response, 'User deleted.')
        except ClientError as e:
            if e.response['Error']['Code'] == 'ResourceNotFoundException':
                log_success(f'{user.username} doesn\'t exist, skipping.')
            else:
                log_error(f'Unable to delete {user.username} user: {e}')
